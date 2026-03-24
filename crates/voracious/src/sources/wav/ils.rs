//! ILS WAV source using shared DSP pipeline.
//!
//! This module provides ILS decoding from pre-demodulated WAV audio files.
//! Unlike the I/Q path which handles RF samples, the WAV path operates on
//! already-demodulated AM audio at 48 kHz.
//!
//! # Processing Pipeline
//!
//! ```text
//! WAV Audio (48 kHz) → Extract envelopes (shared DSP)
//!                   ├─ 90 Hz bandpass → Hilbert → envelope
//!                   ├─ 150 Hz bandpass → Hilbert → envelope
//!                   ├─ Carrier signal → absolute value → 5 Hz lowpass
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

use crate::decoders::ils::{IlsFrame, compute_ddm};
use crate::decoders::morse;
use crate::metrics;
use crate::sources::common::{
    extract_ils_90hz_envelope, extract_ils_150hz_envelope, extract_ils_morse_envelope,
    filter_ils_ddm_smoothing,
};

/// Iterator that reads a WAV audio file and emits [`IlsFrame`] measurements.
///
/// The WAV must be mono PCM containing AM-demodulated ILS audio (typically from GQRX).
/// The sample rate is read from the WAV header — no `--sample-rate` flag needed.
pub struct WavIlsSource {
    /// All samples from the WAV, normalised to −1.0..+1.0.
    samples: Vec<f64>,
    sample_rate: f64,
    timestamp_base: f64,
    ils_frequency: f64,
    window_samples: usize,
    morse_window_samples: usize,
    debug_morse: bool,

    // Pre-computed full-file envelopes (filtfilt applied across the whole file
    // so it has full context — avoids edge artifacts from chunk-by-chunk processing).
    env_90_full: Vec<f64>,
    env_150_full: Vec<f64>,
    carrier_env_full: Vec<f64>,
    morse_envelope: Vec<f64>,

    // Playback state
    pos: usize,
    current_ident: Option<String>,
    all_tokens_history: Vec<String>,
}

impl WavIlsSource {
    /// Open a WAV file for ILS decoding.
    ///
    /// - `ils_freq_mhz`: the ILS localizer frequency (label only, used in JSON output)
    /// - `window_seconds`: DDM averaging window (default 1.0 s)
    /// - `morse_window_seconds`: Morse accumulation window (default 3.0 s)
    /// - `debug_morse`: include candidate list in JSON output
    pub fn new<P: AsRef<Path>>(
        path: P,
        ils_freq_mhz: f64,
        window_seconds: f64,
        morse_window_seconds: f64,
        debug_morse: bool,
    ) -> Result<Self, io::Error> {
        let path_ref = path.as_ref();
        let (raw_samples, sample_rate) = read_wav_mono(path_ref)?;

        // Normalize by maximum amplitude like Python does, then remove residual IF carrier
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

        // Pre-compute the full-file 90/150 Hz envelopes using the shared DSP pipeline.
        // filtfilt needs full signal context to avoid edge artifacts, so we filter
        // the entire file at once, then slice into windows during iteration.
        let env_90_full = extract_ils_90hz_envelope(&samples);
        let env_150_full = extract_ils_150hz_envelope(&samples);

        // Carrier envelope: absolute value of the signal, then 5 Hz lowpass for smoothing
        let carrier_env_raw: Vec<f64> = samples.iter().map(|x| x.abs()).collect();
        let carrier_env_full = filter_ils_ddm_smoothing(&carrier_env_raw);

        // Pre-compute the full-file 1020 Hz Morse envelope.
        let morse_envelope = extract_ils_morse_envelope(&samples);
        let morse_envelope = filter_ils_ddm_smoothing(&morse_envelope);

        Ok(Self {
            samples,
            sample_rate,
            timestamp_base,
            ils_frequency: ils_freq_mhz,
            window_samples,
            morse_window_samples,
            debug_morse,
            env_90_full,
            env_150_full,
            carrier_env_full,
            morse_envelope,
            pos: 0,
            current_ident: None,
            all_tokens_history: Vec::new(),
        })
    }
}

impl Iterator for WavIlsSource {
    type Item = Result<IlsFrame, io::Error>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.pos >= self.samples.len() {
            return None;
        }

        // Slice the pre-computed envelopes at the current window position.
        // This avoids filtfilt edge artifacts from chunk-by-chunk processing.
        let end = (self.pos + self.window_samples).min(self.samples.len());

        let env_90_slice = &self.env_90_full[self.pos..end];
        let env_150_slice = &self.env_150_full[self.pos..end];
        let carrier_slice = &self.carrier_env_full[self.pos..end];

        let ddm_result = compute_ddm(env_90_slice, env_150_slice, carrier_slice);
        let (ddm, mod_90_pct, mod_150_pct, signal_strength) = match ddm_result {
            Ok(result) => (
                result.ddm,
                result.mod_90_hz,
                result.mod_150_hz,
                result.carrier_strength,
            ),
            Err(_) => (0.0, 0.0, 0.0, 0.0),
        };

        // Morse decoding: slice the pre-computed full-file envelope.
        let mut morse_debug = None;
        if end >= self.morse_window_samples {
            let morse_end = end.min(self.morse_envelope.len());
            let morse_start = morse_end.saturating_sub(self.morse_window_samples);
            let morse_slice = &self.morse_envelope[morse_start..morse_end];

            let (ident_single, tokens, attempts) =
                morse::decode_morse_ident(morse_slice, self.sample_rate);
            self.all_tokens_history.extend(tokens);

            // Use the decoded ident if available
            if let Some(ref ident) = ident_single {
                self.current_ident = Some(ident.clone());
            }

            if self.debug_morse {
                let mut counts: HashMap<String, usize> = HashMap::new();
                for token in &self.all_tokens_history {
                    let t = token.to_uppercase();
                    if t.len() == 3 && t.chars().all(|c| c.is_ascii_alphabetic()) {
                        *counts.entry(t).or_insert(0) += 1;
                    }
                }
                let total_count: usize = counts.values().sum();
                let mut candidates: Vec<crate::decoders::ils::IlsMorseCandidate> = counts
                    .into_iter()
                    .map(|(token, count)| crate::decoders::ils::IlsMorseCandidate {
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
                morse_debug = Some(crate::decoders::ils::IlsMorseDebugInfo {
                    candidates,
                    total_tokens: self.all_tokens_history.len(),
                    decode_attempts: attempts,
                });
            }
        }

        let elapsed = end as f64 / self.sample_rate;
        let timestamp = self.timestamp_base + elapsed;

        let frame = IlsFrame::new(
            metrics::round_decimals(timestamp, 5),
            self.ils_frequency,
            metrics::round_decimals(ddm, 4),
            metrics::round_decimals(mod_90_pct, 2),
            metrics::round_decimals(mod_150_pct, 2),
            metrics::round_decimals(signal_strength, 4),
            self.current_ident.clone(),
        )
        .with_morse_debug(morse_debug);

        self.pos = end;

        Some(Ok(frame))
    }
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

/// Parse UTC timestamp from WAV filename (e.g., "111.755_2024-10-24_10-32-28.wav").
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
