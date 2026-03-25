//! VOR WAV source with on-the-fly 30 Hz extraction.
//!
//! This module provides VOR decoding from pre-demodulated WAV audio files.
//! Unlike the I/Q path which handles RF samples, the WAV path operates on
//! already-demodulated AM audio at 48 kHz.
//!
//! # Processing Pipeline
//!
//! ```text
//! WAV Audio (48 kHz) → Extract subcarriers → Extract 30 Hz signals (per chunk)
//!                   ├─ Variable subcarrier (9-11 kHz) → envelope → 30 Hz lowpass
//!                   ├─ Reference subcarrier (9.5-10.5 kHz) → phase differences → 30 Hz lowpass
//!                   └─ 1020 Hz bandpass → Hilbert → envelope → Morse decode
//! ```

use std::f64::consts::PI;
use std::io;
use std::path::Path;
use std::time::{SystemTime, UNIX_EPOCH};

use chrono::NaiveDateTime;
use hound::WavReader;
use num_complex::Complex;
use rustfft::FftPlanner;

use crate::audio::AudioOutput;
use crate::decoders::{
    VOR_30HZ_LPF_CUTOFF, VOR_30HZ_LPF_ORDER, VOR_MORSE_AUDIO_BPF_HIGH, VOR_MORSE_AUDIO_BPF_LOW,
    VOR_MORSE_AUDIO_BPF_ORDER, VOR_REF_SUB_BPF_HIGH, VOR_REF_SUB_BPF_LOW, VOR_REF_SUB_BPF_ORDER,
    VOR_VAR_SUB_BPF_HIGH, VOR_VAR_SUB_BPF_LOW, VOR_VAR_SUB_BPF_ORDER, VorProcessor, VorRadial,
    metrics,
};
use desperado::dsp::filters::ButterworthFilter;
use desperado::dsp::voracious::{envelope, hilbert_transform};

/// Iterator that reads a WAV audio file and emits [`VorRadial`] measurements.
///
/// The WAV must be mono PCM containing AM-demodulated VOR audio (typically from GQRX).
///
/// # On-the-fly 30 Hz Extraction
///
/// Unlike pre-computing var_30/ref_30 for the entire file, this approach extracts them
/// chunk-by-chunk using stateful Butterworth filters. This maintains phase continuity
/// across chunks through the filter states, similar to how the I/Q demodulator works.
pub struct WavVorSource {
    /// Raw audio samples (bipolar, normalised to −1..+1).
    samples: Vec<f64>,
    sample_rate: f64,
    timestamp_base: f64,
    vor_frequency: f64,
    window_samples: usize,

    // Stateful filters for on-the-fly extraction
    var_sub_bpf: ButterworthFilter,
    ref_sub_bpf: ButterworthFilter,
    var_30_lpf: ButterworthFilter,
    ref_30_lpf: ButterworthFilter,
    morse_bpf: ButterworthFilter,

    // Phase tracking state for reference subcarrier FM demodulation
    // Maintains continuity across chunks when unwrapping phase
    last_ref_phase: f64,

    pos: usize,
    processor: VorProcessor,
    audio_output: Option<AudioOutput>,
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

        // Create stateful filters for on-the-fly extraction
        let var_sub_bpf = ButterworthFilter::bandpass(
            VOR_VAR_SUB_BPF_LOW,
            VOR_VAR_SUB_BPF_HIGH,
            sample_rate,
            VOR_VAR_SUB_BPF_ORDER,
        );

        let ref_sub_bpf = ButterworthFilter::bandpass(
            VOR_REF_SUB_BPF_LOW,
            VOR_REF_SUB_BPF_HIGH,
            sample_rate,
            VOR_REF_SUB_BPF_ORDER,
        );

        let var_30_lpf =
            ButterworthFilter::lowpass(VOR_30HZ_LPF_CUTOFF, sample_rate, VOR_30HZ_LPF_ORDER);

        let ref_30_lpf =
            ButterworthFilter::lowpass(VOR_30HZ_LPF_CUTOFF, sample_rate, VOR_30HZ_LPF_ORDER);

        let morse_bpf = ButterworthFilter::bandpass(
            VOR_MORSE_AUDIO_BPF_LOW,
            VOR_MORSE_AUDIO_BPF_HIGH,
            sample_rate,
            VOR_MORSE_AUDIO_BPF_ORDER,
        );

        let processor = VorProcessor::new(
            window_seconds,
            morse_window_seconds,
            sample_rate,
            debug_morse,
        );

        Ok(Self {
            samples,
            sample_rate,
            timestamp_base,
            vor_frequency: vor_freq_mhz,
            window_samples,
            var_sub_bpf,
            ref_sub_bpf,
            var_30_lpf,
            ref_30_lpf,
            morse_bpf,
            last_ref_phase: 0.0,
            pos: 0,
            processor,
            audio_output: None,
        })
    }

    /// Set audio output for real-time audio playback.
    pub fn set_audio_output(&mut self, audio: AudioOutput) {
        self.audio_output = Some(audio);
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
        let chunk = self.samples[self.pos..end].to_vec(); // Clone for processing

        // Stream filtered audio samples to output if enabled
        if let Some(ref audio_out) = self.audio_output {
            let filtered_audio = self.morse_bpf.filter(&chunk);
            for sample in filtered_audio {
                let clamped = sample.clamp(-1.0, 1.0) as f32;
                let _ = audio_out.send(clamped);
            }
        }

        self.pos = end;

        if chunk.is_empty() {
            return None;
        }

        // Extract var_30 and ref_30 on-the-fly from this chunk
        let (var_30, ref_30) = self.extract_vor_30hz_from_chunk(&chunk);

        // Accumulate into processor and emit if ready
        self.processor.accumulate(&chunk, &var_30, &ref_30);

        let elapsed = self.pos as f64 / self.sample_rate;
        let timestamp = self.timestamp_base + elapsed;

        if let Some(output) = self
            .processor
            .emit(timestamp, None, chunk.len(), audio_rate)
        {
            let vor_radial = VorRadial::new(
                metrics::round_decimals(timestamp, 5),
                metrics::round_decimals(output.radial, 2),
                self.vor_frequency,
            )
            .with_quality(output.signal_quality)
            .with_ident(output.ident)
            .with_morse_debug(output.morse_debug);

            Some(Ok(vor_radial))
        } else {
            // Continue accumulating
            self.next()
        }
    }
}

impl WavVorSource {
    /// Extract var_30 and ref_30 from an audio chunk using stateful filters.
    ///
    /// This maintains phase continuity through the filter states across chunks,
    /// similar to how the I/Q demodulator works.
    fn extract_vor_30hz_from_chunk(&mut self, chunk: &Vec<f64>) -> (Vec<f64>, Vec<f64>) {
        // Decimate 48 kHz to ~9 kHz to match I/Q demodulator behavior
        // The subcarrier extraction filters are designed for audio rate, not RF rate
        let decim_factor = 6;
        let decimated: Vec<f64> = chunk.iter().step_by(decim_factor).copied().collect();

        // Extract subcarriers using stateful filters
        let var_sub = self.var_sub_bpf.filter(&decimated);
        let ref_sub = self.ref_sub_bpf.filter(&decimated);

        // Variable: AM demodulation (envelope)
        let var_analytic = hilbert_transform(&var_sub);
        let var_envelope = envelope(&var_analytic);

        // Remove DC from variable signal
        if var_envelope.is_empty() {
            return (Vec::new(), Vec::new());
        }

        let var_envelope_mean = var_envelope.iter().sum::<f64>() / var_envelope.len() as f64;
        let var_centered: Vec<f64> = var_envelope.iter().map(|x| x - var_envelope_mean).collect();
        let var_30 = self.var_30_lpf.filter(&var_centered);

        // Reference: FM demodulation of subcarrier
        let ref_analytic = hilbert_transform(&ref_sub);

        let mut ref_phase_signal = Vec::with_capacity(ref_analytic.len());
        if !ref_analytic.is_empty() {
            // Use stateful phase tracking to maintain continuity across chunks
            let mut prev_phase = self.last_ref_phase;

            for sample in &ref_analytic {
                let phase = sample.arg();
                let mut diff = phase - prev_phase;

                // Unwrap phase
                while diff > PI {
                    diff -= 2.0 * PI;
                }
                while diff < -PI {
                    diff += 2.0 * PI;
                }

                ref_phase_signal.push(diff);
                prev_phase = phase;
            }

            // Save final phase for next chunk
            self.last_ref_phase = prev_phase;
        }

        // Remove DC from reference signal
        if ref_phase_signal.is_empty() {
            return (var_30, Vec::new());
        }

        let ref_mean = ref_phase_signal.iter().sum::<f64>() / ref_phase_signal.len() as f64;
        let ref_centered: Vec<f64> = ref_phase_signal.iter().map(|x| x - ref_mean).collect();
        let ref_30 = self.ref_30_lpf.filter(&ref_centered);

        (var_30, ref_30)
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
