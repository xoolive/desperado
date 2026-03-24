//! VOR WAV source using shared DSP pipeline.
//!
//! This module provides VOR decoding from pre-demodulated WAV audio files.
//! Unlike the I/Q path which handles RF samples, the WAV path operates on
//! already-demodulated AM audio at 48 kHz.
//!
//! # Processing Pipeline
//!
//! ```text
//! WAV Audio (48 kHz) → Extract 30 Hz signals (shared DSP)
//!                   ├─ Variable subcarrier (9-11 kHz) → envelope → 30 Hz lowpass
//!                   ├─ Reference subcarrier (9.5-10.5 kHz) → phase differences → 30 Hz lowpass
//!                   └─ 1020 Hz bandpass → Hilbert → envelope → Morse decode
//! ```

use std::collections::HashMap;
use std::f64::consts::PI;
use std::io;
use std::path::Path;
use std::time::{SystemTime, UNIX_EPOCH};

use chrono::NaiveDateTime;
use hound::WavReader;
use num_complex::Complex;
use rustfft::FftPlanner;

use crate::decoders::vor::VorRadial;
use crate::decoders::{MorseCandidate, MorseDebugInfo, decode_morse_ident};
use crate::metrics;
use crate::sources::common::{
    extract_vor_30hz_modulation, extract_vor_reference_subcarrier_phase,
    extract_vor_variable_subcarrier_envelope,
};

/// Iterator that reads a WAV audio file and emits [`VorRadial`] measurements.
///
/// The WAV must be mono PCM containing AM-demodulated VOR audio (typically from GQRX).
///
/// # Pre-computation of 30 Hz Signals
///
/// The 30 Hz extraction uses Hilbert transforms to obtain analytic signals of the subcarriers.
/// When applied chunk-by-chunk, phase discontinuities occur at boundaries because each chunk's
/// FFT-based Hilbert produces phase relative to that chunk alone. Processing the full signal
/// in one pass avoids this entirely. Thus, var_30/ref_30 are pre-computed over the entire file.
pub struct WavVorSource {
    /// Raw audio samples (bipolar, normalised to −1..+1).
    samples: Vec<f64>,
    /// Pre-computed var_30 signal for the full recording.
    var_30: Vec<f64>,
    /// Pre-computed ref_30 signal for the full recording.
    ref_30: Vec<f64>,
    sample_rate: f64,
    timestamp_base: f64,
    vor_frequency: f64,
    window_samples: usize,
    morse_window_samples: usize,
    debug_morse: bool,

    pos: usize,
    morse_audio_buf: Vec<f64>,
    ident_votes: HashMap<String, usize>,
    ident_hit_timestamps: HashMap<String, Vec<f64>>,
    radial_history: Vec<f64>,
    windows_total: usize,
    all_tokens_history: Vec<String>,
}

impl WavVorSource {
    /// Open a WAV file for VOR decoding.
    ///
    /// - `vor_freq_mhz`: the VOR frequency (label only, used in JSON output)
    /// - `window_seconds`: Radial averaging window (default 3.0 s)
    /// - `morse_window_seconds`: Morse accumulation window (default 15.0 s)
    /// - `debug_morse`: include candidate list in JSON output
    pub fn new<P: AsRef<Path>>(
        path: P,
        vor_freq_mhz: f64,
        window_seconds: f64,
        morse_window_seconds: f64,
        debug_morse: bool,
    ) -> Result<Self, io::Error> {
        let path_ref = path.as_ref();
        let (raw_samples, sample_rate) = read_wav_mono(path_ref)?;

        // Normalize by maximum amplitude
        let max_amp = raw_samples.iter().map(|x| x.abs()).fold(0.0f64, f64::max);
        let normalized_samples: Vec<f64> = if max_amp > 1e-9 {
            raw_samples.iter().map(|x| x / max_amp).collect()
        } else {
            raw_samples
        };

        let (samples, _carrier) = demod_real_if(&normalized_samples, sample_rate);

        let timestamp_base = parse_wav_start_unix(path_ref).unwrap_or_else(unix_now_seconds);
        let window_samples = (window_seconds * sample_rate).round() as usize;
        let morse_window_samples = (morse_window_seconds * sample_rate).round() as usize;

        // Pre-compute var_30 and ref_30 for the full file using the shared DSP pipeline.
        // This avoids phase discontinuities from chunk-by-chunk Hilbert transforms.
        let (var_30, ref_30) = extract_vor_30hz_signals(&samples);

        Ok(Self {
            samples,
            var_30,
            ref_30,
            sample_rate,
            timestamp_base,
            vor_frequency: vor_freq_mhz,
            window_samples,
            morse_window_samples,
            debug_morse,
            pos: 0,
            morse_audio_buf: Vec::new(),
            ident_votes: HashMap::new(),
            ident_hit_timestamps: HashMap::new(),
            radial_history: Vec::new(),
            windows_total: 0,
            all_tokens_history: Vec::new(),
        })
    }
}

impl Iterator for WavVorSource {
    type Item = Result<VorRadial, io::Error>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.pos >= self.samples.len() {
            return None;
        }

        let audio_rate = self.sample_rate;
        let end = (self.pos + self.window_samples).min(self.samples.len());
        let chunk = &self.samples[self.pos..end];
        let var_30_win = &self.var_30[self.pos.min(self.var_30.len())..end.min(self.var_30.len())];
        let ref_30_win = &self.ref_30[self.pos.min(self.ref_30.len())..end.min(self.ref_30.len())];

        self.morse_audio_buf.extend_from_slice(chunk);
        self.pos = end;

        if chunk.is_empty() {
            return None;
        }

        // Calculate radial from the pre-computed var_30/ref_30 for this window.
        let radial = crate::decoders::calculate_radial(var_30_win, ref_30_win, audio_rate);

        if let Some(radial) = radial {
            self.radial_history.push(radial);
            if self.radial_history.len() > 20 {
                self.radial_history.remove(0);
            }
        }

        self.windows_total += 1;
        let elapsed = self.pos as f64 / self.sample_rate;
        let timestamp = self.timestamp_base + elapsed;

        let mut ident_detected_now: Option<String> = None;
        let mut morse_debug_info: Option<MorseDebugInfo> = None;

        if self.morse_audio_buf.len() >= self.morse_window_samples {
            let (candidate, tokens, attempts) =
                decode_morse_ident(&self.morse_audio_buf, audio_rate);
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

            if self.debug_morse {
                let mut counts: HashMap<String, usize> = HashMap::new();
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

                morse_debug_info = Some(MorseDebugInfo {
                    candidates,
                    total_tokens: self.all_tokens_history.len(),
                    windows_total: self.windows_total,
                    ident_hits_seconds,
                    repeat_interval_seconds,
                    next_expected_seconds,
                    decode_attempts: attempts,
                });
            }

            let keep = self.morse_window_samples / 2;
            if self.morse_audio_buf.len() > self.morse_window_samples + keep {
                self.morse_audio_buf.drain(0..keep);
            }
        }

        let signal_quality = metrics::compute_signal_quality(
            None,
            chunk,
            var_30_win,
            ref_30_win,
            audio_rate,
            chunk.len(),
            &self.radial_history,
        );

        let radial_deg = self.radial_history.last().copied().unwrap_or(0.0);

        let vor_radial = VorRadial::new(
            metrics::round_decimals(timestamp, 5),
            metrics::round_decimals(radial_deg, 2),
            self.vor_frequency,
        )
        .with_quality(signal_quality)
        .with_ident(ident_detected_now)
        .with_morse_debug(morse_debug_info);

        Some(Ok(vor_radial))
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// VOR 30 Hz signal extraction using shared DSP
// ──────────────────────────────────────────────────────────────────────────────

/// Extract var_30 and ref_30 signals from WAV audio for VOR decoding.
///
/// The WAV audio is the AM-demodulated baseband signal at 48 kHz, containing:
/// - Variable signal: 30 Hz modulating the 9000-11000 Hz subcarrier (AM)
/// - Reference signal: 30 Hz modulating the 9500-10500 Hz subcarrier (FM)
///
/// Uses the shared DSP functions from `sources::common` to extract subcarriers and
/// modulation signals consistently with the I/Q path.
///
/// Returns (var_30, ref_30) - both at 48 kHz sample rate
fn extract_vor_30hz_signals(audio: &[f64]) -> (Vec<f64>, Vec<f64>) {
    // Extract variable subcarrier envelope (9000-11000 Hz, AM-modulated)
    let var_envelope = extract_vor_variable_subcarrier_envelope(audio);

    // Remove DC from variable envelope
    let var_mean = var_envelope.iter().sum::<f64>() / var_envelope.len() as f64;
    let var_centered: Vec<f64> = var_envelope.iter().map(|x| x - var_mean).collect();

    // Extract reference subcarrier analytic signal (9500-10500 Hz, FM-modulated)
    let ref_analytic = extract_vor_reference_subcarrier_phase(audio);

    // FM demodulation: compute phase differences with unwrapping
    let mut ref_phase_signal = Vec::with_capacity(ref_analytic.len());
    if !ref_analytic.is_empty() {
        let mut prev_phase = ref_analytic[0].arg();

        for sample in &ref_analytic {
            let phase = sample.arg();
            let mut diff = phase - prev_phase;

            // Unwrap: keep phase difference in [-π, π]
            while diff > PI {
                diff -= 2.0 * PI;
            }
            while diff < -PI {
                diff += 2.0 * PI;
            }

            ref_phase_signal.push(diff);
            prev_phase = phase;
        }
    }

    // Remove DC from reference signal
    let ref_mean = ref_phase_signal.iter().sum::<f64>() / ref_phase_signal.len() as f64;
    let ref_centered: Vec<f64> = ref_phase_signal.iter().map(|x| x - ref_mean).collect();

    // Apply 30 Hz lowpass to smooth (using shared DSP)
    let var_30 = extract_vor_30hz_modulation(&var_centered);
    let ref_30 = extract_vor_30hz_modulation(&ref_centered);

    (var_30, ref_30)
}

// ──────────────────────────────────────────────────────────────────────────────
// WAV file reading utilities
// ──────────────────────────────────────────────────────────────────────────────

/// Read a WAV file and return mono samples normalized to float.
fn read_wav_mono<P: AsRef<Path>>(path: P) -> Result<(Vec<f64>, f64), io::Error> {
    let reader = WavReader::open(&path)
        .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e.to_string()))?;

    let spec = reader.spec();
    if spec.channels != 1 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("Expected mono WAV, got {} channels", spec.channels),
        ));
    }

    let sample_rate = spec.sample_rate as f64;
    let samples: Result<Vec<f64>, _> = reader
        .into_samples::<i16>()
        .map(|s| {
            s.map(|sample| sample as f64 / 32768.0)
                .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e.to_string()))
        })
        .collect();

    let samples = samples?;
    Ok((samples, sample_rate))
}

/// Parse UTC timestamp from WAV filename (e.g., "114.850_2024-10-24_10-32-28.wav").
fn parse_wav_start_unix<P: AsRef<Path>>(path: P) -> Option<f64> {
    let filename = path.as_ref().file_name()?.to_string_lossy();
    let name_without_ext = filename.split('.').next()?;
    let parts: Vec<&str> = name_without_ext.split('_').collect();
    if parts.len() < 3 {
        return None;
    }

    let date_part = parts[1];
    let time_part = parts[2];
    let datetime_str = format!("{} {}", date_part, time_part.replace('-', ":"));

    let dt = NaiveDateTime::parse_from_str(&datetime_str, "%Y-%m-%d %H:%M:%S").ok()?;
    Some(dt.and_utc().timestamp() as f64)
}

/// Get current time as Unix timestamp (seconds since epoch).
fn unix_now_seconds() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}

/// Detect and suppress a residual IF carrier from a WAV recording.
fn demod_real_if(samples: &[f64], sample_rate: f64) -> (Vec<f64>, Option<f64>) {
    // Spectrum analysis for carrier detection
    let fft_len = samples.len().min(1 << 17).next_power_of_two();
    let mut planner = FftPlanner::<f64>::new();
    let fft = planner.plan_fft_forward(fft_len);

    let mut spectrum: Vec<Complex<f64>> = samples[..fft_len]
        .iter()
        .map(|&s| Complex::new(s, 0.0))
        .collect();
    fft.process(&mut spectrum);

    // Power spectrum (one-sided)
    let num_bins = fft_len / 2;
    let power: Vec<f64> = spectrum[..num_bins].iter().map(|c| c.norm_sqr()).collect();
    let bin_hz = sample_rate / fft_len as f64;

    // Find dominant peak above 200 Hz
    let min_bin = (200.0 / bin_hz).ceil() as usize;
    let (peak_bin, &peak_power) = power[min_bin..]
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
        .map(|(i, p)| (i + min_bin, p))
        .unwrap_or((min_bin, &0.0));

    // Check if peak is significant
    let mut sorted_power = power.clone();
    sorted_power.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let median_power = sorted_power[sorted_power.len() / 2];

    let threshold_linear = median_power * 100.0; // 20 dB above median
    if peak_power < threshold_linear || median_power == 0.0 {
        return (samples.to_vec(), None);
    }

    // Refine carrier frequency
    let lo = peak_bin.saturating_sub(2);
    let hi = (peak_bin + 3).min(num_bins);
    let (weighted_freq, weight_sum) =
        power[lo..hi]
            .iter()
            .enumerate()
            .fold((0.0_f64, 0.0_f64), |(wf, ws), (i, &p)| {
                let freq = (lo + i) as f64 * bin_hz;
                (wf + freq * p, ws + p)
            });
    let carrier_freq = if weight_sum > 0.0 {
        weighted_freq / weight_sum
    } else {
        peak_bin as f64 * bin_hz
    };

    // Apply notch filter
    let q = 30.0_f64;
    let w0 = 2.0 * PI * carrier_freq / sample_rate;
    let alpha = w0.sin() / (2.0 * q);
    let cos_w0 = w0.cos();

    // Notch biquad coefficients
    let b0 = 1.0;
    let b1 = -2.0 * cos_w0;
    let b2 = 1.0;
    let a1 = -2.0 * cos_w0;
    let a2 = 1.0 - alpha;

    // Forward pass
    let mut filtered = vec![0.0; samples.len()];
    let mut y1 = 0.0;
    let mut y2 = 0.0;
    let mut x1 = 0.0;
    let mut x2 = 0.0;

    for (i, &x) in samples.iter().enumerate() {
        let y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        filtered[i] = y;
        x2 = x1;
        x1 = x;
        y2 = y1;
        y1 = y;
    }

    // Backward pass (zero-phase)
    let reversed: Vec<f64> = filtered.iter().rev().cloned().collect();
    let mut backward = vec![0.0; samples.len()];
    y1 = 0.0;
    y2 = 0.0;
    x1 = 0.0;
    x2 = 0.0;

    for (i, &x) in reversed.iter().enumerate() {
        let y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        backward[i] = y;
        x2 = x1;
        x1 = x;
        y2 = y1;
        y1 = y;
    }

    backward.reverse();
    (backward, Some(carrier_freq))
}
