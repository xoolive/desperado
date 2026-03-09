//! I/Q sources built on desperado readers

use std::collections::HashMap;
use std::io;
use std::path::Path;
use std::str::FromStr;
use std::time::{SystemTime, UNIX_EPOCH};

use chrono::NaiveDateTime;
#[cfg(feature = "airspy")]
use desperado::Gain;
use desperado::{
    DeviceConfig, IqAsyncSource as BaseIqAsyncSource, IqFormat, IqSource as BaseIqSource,
};
#[cfg(feature = "airspy")]
use futures::StreamExt;

use crate::decode::{
    decode_morse_ident, MorseCandidate, MorseDebugInfo, SignalQualityMetrics, VorRadial,
};
use crate::dsp::VorDemodulator;

const DEFAULT_CHUNK_SAMPLES: usize = 262_144;

pub type IqAsyncSource = BaseIqAsyncSource;

enum SourceInner {
    Sync(BaseIqSource),
    #[cfg(feature = "airspy")]
    Async {
        runtime: tokio::runtime::Runtime,
        source: BaseIqAsyncSource,
    },
}

enum TimestampBase {
    FileStartUnix(f64),
    LiveWallClock,
}

pub struct IqSource {
    source: SourceInner,
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
}

impl IqSource {
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
            let file_source = BaseIqSource::from_file(
                path_ref,
                center_freq_hz,
                sample_rate,
                DEFAULT_CHUNK_SAMPLES,
                format,
            )
            .map_err(|e| io::Error::other(e.to_string()))?;
            SourceInner::Sync(file_source)
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
        })
    }
}

impl Iterator for IqSource {
    type Item = Result<VorRadial, io::Error>;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            let samples = match &mut self.source {
                SourceInner::Sync(source) => match source.next() {
                    Some(Ok(s)) if s.is_empty() => return None,
                    Some(Ok(s)) => s,
                    Some(Err(e)) => return Some(Err(io::Error::other(e.to_string()))),
                    None => return None,
                },
                #[cfg(feature = "airspy")]
                SourceInner::Async { runtime, source } => match runtime.block_on(source.next()) {
                    Some(Ok(s)) if s.is_empty() => return None,
                    Some(Ok(s)) => s,
                    Some(Err(e)) => return Some(Err(io::Error::other(e.to_string()))),
                    None => return None,
                },
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
            self.sample_count += samples.len();

            // Check if we have enough for a radial calculation window
            if self.audio_buffer.len() >= self.window_samples {
                let audio_rate = self.demodulator.audio_rate();
                if let Some(radial) =
                    crate::dsp::vor::calculate_radial_vortrack(&self.audio_buffer, audio_rate)
                {
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
                                        .and_then(|hits| estimate_repeat_interval_seconds(hits))
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
                                            let interval = estimate_repeat_interval_seconds(hits)?;
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

                    let signal_quality = compute_signal_quality(
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
                        round_decimals(timestamp, 5),
                        round_decimals(radial, 2),
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
) -> Result<SourceInner, io::Error> {
    if uri.starts_with("airspy://") {
        return build_airspy_source(uri, center_freq_hz, sample_rate);
    }

    let configured_uri = ensure_tuning_query(uri, center_freq_hz, sample_rate);
    let config =
        DeviceConfig::from_str(&configured_uri).map_err(|e| io::Error::other(e.to_string()))?;
    let source =
        BaseIqSource::from_device_config(config).map_err(|e| io::Error::other(e.to_string()))?;
    Ok(SourceInner::Sync(source))
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
) -> Result<SourceInner, io::Error> {
    let rest = &uri["airspy://".len()..];
    let (device_part, query) = if let Some(pos) = rest.find('?') {
        (&rest[..pos], &rest[pos + 1..])
    } else {
        (rest, "")
    };

    let device_index =
        if device_part.is_empty() {
            None
        } else {
            Some(device_part.parse::<usize>().map_err(|_| {
                io::Error::other(format!("Invalid airspy device index: {device_part}"))
            })?)
        };

    let mut gain = Gain::Auto;
    let mut lna_gain: Option<u8> = None;
    let mut mixer_gain: Option<u8> = None;
    let mut vga_gain: Option<u8> = None;

    for param in query.split('&') {
        if param.is_empty() {
            continue;
        }
        let kv: Vec<&str> = param.splitn(2, '=').collect();
        if kv.len() != 2 {
            continue;
        }
        match kv[0] {
            "gain" => {
                gain =
                    if kv[1].eq_ignore_ascii_case("auto") {
                        Gain::Auto
                    } else {
                        Gain::Manual(kv[1].parse::<f64>().map_err(|_| {
                            io::Error::other(format!("Invalid airspy gain: {}", kv[1]))
                        })?)
                    };
            }
            "lna" | "lna_gain" => {
                lna_gain = Some(kv[1].parse::<u8>().map_err(|_| {
                    io::Error::other(format!("Invalid airspy lna gain: {}", kv[1]))
                })?);
            }
            "mixer" | "mixer_gain" => {
                mixer_gain = Some(kv[1].parse::<u8>().map_err(|_| {
                    io::Error::other(format!("Invalid airspy mixer gain: {}", kv[1]))
                })?);
            }
            "vga" | "vga_gain" => {
                vga_gain = Some(kv[1].parse::<u8>().map_err(|_| {
                    io::Error::other(format!("Invalid airspy vga gain: {}", kv[1]))
                })?);
            }
            _ => {}
        }
    }

    let runtime = tokio::runtime::Runtime::new().map_err(|e| io::Error::other(e.to_string()))?;
    let source = runtime
        .block_on(BaseIqAsyncSource::from_airspy(
            device_index,
            center_freq_hz,
            sample_rate,
            gain,
            lna_gain,
            mixer_gain,
            vga_gain,
        ))
        .map_err(|e| io::Error::other(e.to_string()))?;

    Ok(SourceInner::Async { runtime, source })
}

#[cfg(not(feature = "airspy"))]
fn build_airspy_source(
    _uri: &str,
    _center_freq_hz: u32,
    _sample_rate: u32,
) -> Result<SourceInner, io::Error> {
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

fn compute_signal_quality(
    clipping_ratio: Option<f64>,
    audio: &[f64],
    var_30: &[f64],
    ref_30: &[f64],
    sample_rate: f64,
    window_iq_count: usize,
    radial_history: &[f64],
) -> SignalQualityMetrics {
    let snr_9960_db = tone_snr_db(
        audio,
        sample_rate,
        9960.0,
        &[9600.0, 9700.0, 9800.0, 10100.0, 10200.0, 10300.0],
    );

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

    let lock_score = phase_lock_score(var_30, ref_30);

    let radial_var = circular_variance(radial_history);

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

fn round_decimals(value: f64, decimals: u32) -> f64 {
    let factor = 10_f64.powi(decimals as i32);
    (value * factor).round() / factor
}

fn format_scientific(value: f64, decimals: usize) -> String {
    format!("{value:.decimals$e}")
}

fn rms_power(signal: &[f64]) -> f64 {
    if signal.is_empty() {
        return 0.0;
    }
    signal.iter().map(|x| x * x).sum::<f64>() / signal.len() as f64
}

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

fn tone_snr_db(signal: &[f64], sample_rate: f64, tone_freq: f64, noise_freqs: &[f64]) -> f64 {
    let p_sig = tone_power(signal, sample_rate, tone_freq).max(1e-12);
    let p_noise = avg_tone_power(signal, sample_rate, noise_freqs).max(1e-12);
    10.0 * (p_sig / p_noise).log10()
}

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

fn estimate_repeat_interval_seconds(hits: &[f64]) -> Option<f64> {
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
