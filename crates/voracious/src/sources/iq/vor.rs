//! I/Q source handling for VOR signal processing
//!
//! This module provides unified input handling for both file and live device sources,
//! with integrated VOR demodulation and signal quality analysis.
//!
//! # Overview
//!
//! The `VorSource` struct wraps synchronous file I/O or device access
//! (e.g., Airspy, RTL-SDR, SoapySDR)
//!
//! # Input Sources
//!
//! Supports two classes of input:
//!
//! ## File-based Input
//! - GQRX `.raw` recordings (complex float, auto-detected sample rate & center frequency from filename)
//! - Raw I/Q binary files (cu8, cs8, cs16, cf32 formats supported)
//!
//! ## Live Devices (URI scheme)
//! - `rtlsdr://[?freq=...&rate=...]` - RTL-SDR dongles
//! - `soapy://[?freq=...&rate=...]` - SoapySDR compatible devices
//! - `airspy://[device_idx][?freq=...&rate=...&lna=...&mixer=...&vga=...]` - Airspy HF+ / Mini
//!
//! # Processing Pipeline
//!
//! ```text
//! Input (File/Device) → I/Q Samples → Demodulator → Audio (3 kHz)
//!                                                    └─ Variable 30 Hz signal
//!                                                    └─ Reference 30 Hz signal
//!                                                    └─ Morse audio buffer
//!
//! Audio Buffer (3 samples) → Radial Calculation → Signal Quality Metrics
//!                          → Morse Decoding → Ident Detection
//!                          → VorRadial Output
//! ```

use std::collections::HashMap;
use std::io;
use std::path::Path;
use std::str::FromStr;
use std::time::{SystemTime, UNIX_EPOCH};

use chrono::NaiveDateTime;
#[cfg(feature = "airspy")]
use desperado::Gain;
use desperado::{DeviceConfig, IqFormat, IqSource as BaseIqSource};

use crate::audio::AudioOutput;
use crate::decoders::{
    MorseCandidate, MorseDebugInfo, VorDemodulator, VorRadial, calculate_radial_vortrack,
    decode_morse_ident,
};
use crate::metrics;

const DEFAULT_CHUNK_SAMPLES: usize = 262_144;

enum TimestampBase {
    FileStartUnix(f64),
    LiveWallClock,
}

pub struct VorSource {
    source: BaseIqSource,
    demodulator: VorDemodulator,
    vor_frequency: f64,
    center_frequency: f64,
    sample_count: usize,
    window_samples: usize,
    morse_window_samples: usize,
    audio_buffer: Vec<f64>,
    var_30_buffer: Vec<f64>,
    ref_30_buffer: Vec<f64>,
    morse_audio_buffer: Vec<f64>,
    ident_votes: HashMap<String, usize>,
    ident_hit_timestamps: HashMap<String, Vec<f64>>,
    radial_history: Vec<f64>,
    window_iq_count: usize,
    window_clip_count: usize,
    windows_total: usize,
    debug_morse: bool,
    all_tokens_history: Vec<String>,
    timestamp_base: TimestampBase,
    audio_output: Option<AudioOutput>,
}

impl VorSource {
    #[allow(clippy::too_many_arguments)]
    pub fn new<P: AsRef<Path>>(
        path: P,
        sample_rate: u32,
        format: IqFormat,
        vor_freq_mhz: f64,
        center_freq_mhz: f64,
        window_seconds: f64,
        morse_window_seconds: f64,
        debug_morse: bool,
    ) -> Result<Self, io::Error> {
        let path_ref = path.as_ref();
        let input = path_ref.to_string_lossy().to_string();
        let center_freq_hz = (center_freq_mhz * 1e6).round() as u32;
        let is_live = is_device_uri(&input);
        let source = if is_live {
            build_device_source(&input, center_freq_hz, sample_rate)?
        } else {
            BaseIqSource::from_file(
                path_ref,
                center_freq_hz,
                sample_rate,
                DEFAULT_CHUNK_SAMPLES,
                format,
            )
            .map_err(|e| io::Error::other(e.to_string()))?
        };
        let timestamp_base = if is_live {
            TimestampBase::LiveWallClock
        } else {
            TimestampBase::FileStartUnix(resolve_file_start_unix(path_ref)?)
        };

        let demodulator = VorDemodulator::new(sample_rate);
        let audio_rate = demodulator.audio_rate();
        let window_samples = (window_seconds * audio_rate) as usize;
        let morse_window_samples = (morse_window_seconds * audio_rate) as usize;

        Ok(Self {
            source,
            demodulator,
            vor_frequency: vor_freq_mhz * 1e6,
            center_frequency: center_freq_mhz * 1e6,
            sample_count: 0,
            window_samples,
            morse_window_samples,
            audio_buffer: Vec::new(),
            var_30_buffer: Vec::new(),
            ref_30_buffer: Vec::new(),
            morse_audio_buffer: Vec::new(),
            ident_votes: HashMap::new(),
            ident_hit_timestamps: HashMap::new(),
            radial_history: Vec::new(),
            window_iq_count: 0,
            window_clip_count: 0,
            windows_total: 0,
            debug_morse,
            all_tokens_history: Vec::new(),
            timestamp_base,
            audio_output: None,
        })
    }

    /// Set audio output for morse ident playback.
    pub fn set_audio_output(&mut self, audio: AudioOutput) {
        self.audio_output = Some(audio);
    }
}

impl Iterator for VorSource {
    type Item = Result<VorRadial, io::Error>;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            let samples = match self.source.next() {
                Some(Ok(s)) if s.is_empty() => return None,
                Some(Ok(s)) => s,
                Some(Err(e)) => return Some(Err(io::Error::other(e.to_string()))),
                None => return None,
            };

            let freq_offset = self.vor_frequency - self.center_frequency;
            for sample in &samples {
                self.window_iq_count += 1;
                if sample.re.abs() >= 0.98 || sample.im.abs() >= 0.98 {
                    self.window_clip_count += 1;
                }
            }

            let (var_30, ref_30, audio) = self.demodulator.demodulate(&samples, freq_offset);

            self.audio_buffer.extend(&audio);
            self.var_30_buffer.extend(&var_30);
            self.ref_30_buffer.extend(&ref_30);
            self.morse_audio_buffer.extend(&audio);

            // Stream audio samples to soundcard if audio output is enabled
            if let Some(ref audio_out) = self.audio_output {
                // Normalize to f32 range [-1.0, 1.0] and send to audio output
                // Normalize based on typical signal envelope (mean ~0.5)
                for sample in &audio {
                    let normalized = (sample / 0.5).clamp(-1.0, 1.0) as f32;
                    let _ = audio_out.send(normalized);
                }
            }

            self.sample_count += samples.len();

            // Check if we have enough for a radial calculation window
            if self.audio_buffer.len() >= self.window_samples {
                let audio_rate = self.demodulator.audio_rate();
                if let Some(radial) = calculate_radial_vortrack(&self.audio_buffer, audio_rate) {
                    self.radial_history.push(radial);
                    if self.radial_history.len() > 20 {
                        self.radial_history.remove(0);
                    }

                    self.windows_total += 1;
                    let elapsed = self.sample_count as f64 / self.demodulator.sample_rate;
                    let timestamp = match self.timestamp_base {
                        TimestampBase::FileStartUnix(t0) => t0 + elapsed,
                        TimestampBase::LiveWallClock => unix_now_seconds(),
                    };
                    let mut ident_detected_now: Option<String> = None;

                    // Try Morse decoding if we have enough accumulated audio
                    let morse_debug = if self.morse_audio_buffer.len() >= self.morse_window_samples
                    {
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
                        if self.morse_audio_buffer.len() > self.morse_window_samples + keep_samples
                        {
                            self.morse_audio_buffer.drain(0..keep_samples);
                        }

                        // Build debug info if requested
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

                            candidates.sort_by(|a, b| {
                                b.count.cmp(&a.count).then_with(|| a.token.cmp(&b.token))
                            });

                            Some(MorseDebugInfo {
                                candidates,
                                total_tokens: self.all_tokens_history.len(),
                                windows_total: self.windows_total,
                                ident_hits_seconds: {
                                    let focus_ident = self
                                        .ident_votes
                                        .iter()
                                        .max_by_key(|(_, count)| **count)
                                        .map(|(ident, _)| ident.clone());
                                    focus_ident
                                        .and_then(|id| self.ident_hit_timestamps.get(&id).cloned())
                                        .unwrap_or_default()
                                },
                                repeat_interval_seconds: {
                                    let focus_ident = self
                                        .ident_votes
                                        .iter()
                                        .max_by_key(|(_, count)| **count)
                                        .map(|(ident, _)| ident.clone());
                                    focus_ident
                                        .and_then(|id| self.ident_hit_timestamps.get(&id))
                                        .and_then(|hits| {
                                            metrics::estimate_repeat_interval_seconds(hits)
                                        })
                                },
                                next_expected_seconds: {
                                    let focus_ident = self
                                        .ident_votes
                                        .iter()
                                        .max_by_key(|(_, count)| **count)
                                        .map(|(ident, _)| ident.clone());
                                    focus_ident
                                        .and_then(|id| self.ident_hit_timestamps.get(&id))
                                        .and_then(|hits| {
                                            let interval =
                                                metrics::estimate_repeat_interval_seconds(hits)?;
                                            let last = hits.last().copied()?;
                                            Some(last + interval)
                                        })
                                },
                                decode_attempts: attempts,
                            })
                        } else {
                            None
                        }
                    } else {
                        None
                    };

                    let ident = ident_detected_now.clone();

                    let window_clipping_ratio = if self.window_iq_count > 0 {
                        Some(self.window_clip_count as f64 / self.window_iq_count as f64)
                    } else {
                        None
                    };

                    let signal_quality = metrics::compute_signal_quality(
                        window_clipping_ratio,
                        &self.audio_buffer,
                        &self.var_30_buffer,
                        &self.ref_30_buffer,
                        audio_rate,
                        self.window_iq_count,
                        &self.radial_history,
                    );

                    self.audio_buffer.clear();
                    self.var_30_buffer.clear();
                    self.ref_30_buffer.clear();
                    self.window_iq_count = 0;
                    self.window_clip_count = 0;

                    let vor_radial = VorRadial::new(
                        metrics::round_decimals(timestamp, 5),
                        metrics::round_decimals(radial, 2),
                        self.vor_frequency / 1e6,
                    )
                    .with_quality(signal_quality)
                    .with_ident(ident)
                    .with_morse_debug(morse_debug);
                    return Some(Ok(vor_radial));
                }

                self.audio_buffer.clear();
                self.var_30_buffer.clear();
                self.ref_30_buffer.clear();
                self.window_iq_count = 0;
                self.window_clip_count = 0;
            }
        }
    }
}

fn is_device_uri(input: &str) -> bool {
    input.starts_with("rtlsdr://")
        || input.starts_with("soapy://")
        || input.starts_with("airspy://")
}

fn build_device_source(
    uri: &str,
    center_freq_hz: u32,
    sample_rate: u32,
) -> Result<BaseIqSource, io::Error> {
    if uri.starts_with("airspy://") {
        return build_airspy_source(uri, center_freq_hz, sample_rate);
    }

    let configured_uri = ensure_tuning_query(uri, center_freq_hz, sample_rate);
    let config =
        DeviceConfig::from_str(&configured_uri).map_err(|e| io::Error::other(e.to_string()))?;
    BaseIqSource::from_device_config(config).map_err(|e| io::Error::other(e.to_string()))
}

fn ensure_tuning_query(uri: &str, center_freq_hz: u32, sample_rate: u32) -> String {
    let has_query = uri.contains('?');
    let has_freq = uri.contains("freq=") || uri.contains("frequency=");
    let has_rate = uri.contains("rate=") || uri.contains("sample_rate=");

    let mut out = uri.to_string();
    if !has_query {
        out.push('?');
    }
    if !has_freq {
        if !out.ends_with('?') && !out.ends_with('&') {
            out.push('&');
        }
        out.push_str(&format!("freq={center_freq_hz}"));
    }
    if !has_rate {
        if !out.ends_with('?') && !out.ends_with('&') {
            out.push('&');
        }
        out.push_str(&format!("rate={sample_rate}"));
    }
    out
}

#[cfg(feature = "airspy")]
fn build_airspy_source(
    uri: &str,
    center_freq_hz: u32,
    sample_rate: u32,
) -> Result<BaseIqSource, io::Error> {
    let rest = &uri["airspy://".len()..];
    let (device_part, query) = if let Some(pos) = rest.find('?') {
        (&rest[..pos], &rest[pos + 1..])
    } else {
        (rest, "")
    };

    let device_index = if device_part.is_empty() {
        0
    } else {
        device_part
            .parse::<usize>()
            .map_err(|_| io::Error::other(format!("Invalid airspy device index: {device_part}")))?
    };

    let mut gain = Gain::Auto;

    for param in query.split('&') {
        if param.is_empty() {
            continue;
        }
        let kv: Vec<&str> = param.splitn(2, '=').collect();
        if kv.len() != 2 {
            continue;
        }
        if kv[0] == "gain" {
            gain = if kv[1].eq_ignore_ascii_case("auto") {
                Gain::Auto
            } else {
                Gain::Manual(
                    kv[1]
                        .parse::<f64>()
                        .map_err(|_| io::Error::other(format!("Invalid airspy gain: {}", kv[1])))?,
                )
            };
        }
        // Note: lna_gain, mixer_gain, vga_gain are part of AirspyConfig but
        // currently not parsed from URI for simplicity
    }

    let config =
        desperado::airspy::AirspyConfig::new(device_index, center_freq_hz, sample_rate, gain);
    let source = desperado::airspy::AirspySdrReader::new(&config)
        .map_err(|e| io::Error::other(e.to_string()))?;

    Ok(BaseIqSource::Airspy(source))
}

#[cfg(not(feature = "airspy"))]
fn build_airspy_source(
    _uri: &str,
    _center_freq_hz: u32,
    _sample_rate: u32,
) -> Result<BaseIqSource, io::Error> {
    Err(io::Error::other(
        "airspy:// is not enabled. Rebuild with --features airspy",
    ))
}

fn unix_now_seconds() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}

fn resolve_file_start_unix(path: &Path) -> Result<f64, io::Error> {
    if let Some(ts) = parse_gqrx_start_unix(path) {
        return Ok(ts);
    }

    let meta = std::fs::metadata(path)?;
    if let Ok(created) = meta.created() {
        return Ok(created
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_secs_f64())
            .unwrap_or(0.0));
    }

    if let Ok(modified) = meta.modified() {
        return Ok(modified
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_secs_f64())
            .unwrap_or(0.0));
    }

    Err(io::Error::other(
        "Could not determine absolute start time for input file",
    ))
}

fn parse_gqrx_start_unix(path: &Path) -> Option<f64> {
    let name = path.file_name()?.to_str()?;
    if !name.starts_with("gqrx_") {
        return None;
    }

    let mut parts = name.split('_');
    if parts.next()? != "gqrx" {
        return None;
    }

    let date = parts.next()?;
    let time = parts.next()?;
    if date.len() != 8 || time.len() != 6 {
        return None;
    }

    let dt = NaiveDateTime::parse_from_str(&format!("{date}{time}"), "%Y%m%d%H%M%S").ok()?;
    Some(dt.and_utc().timestamp() as f64)
}
