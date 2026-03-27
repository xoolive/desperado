//! I/Q source handling for ILS localizer signal processing.
//!
//! Mirrors the structure of the VOR module but drives [`IlsLocalizerDemodulator`]
//! and emits [`IlsFrame`] values instead of `VorRadial`.
//!
//! # Processing Pipeline
//!
//! ```text
//! cf32 IQ file / device
//!   → freq-shift (carrier offset)
//!   → AM envelope
//!   → decimate 200× → 9 kHz audio rate
//!   ├─ BPF 80–100 Hz  → Hilbert envelope → env_90  ┐
//!   └─ BPF 140–160 Hz → Hilbert envelope → env_150 ┘→ DDM per window
//!   └─ BPF 900–1100 Hz → envelope → Morse ident (every 15 s)
//! ```
//!
//! # Output
//!
//! One [`IlsFrame`] per 1-second audio window (9000 samples at 9 kHz).

use std::collections::HashMap;
use std::io;
use std::path::Path;

use desperado::{IqFormat, IqSource as BaseIqSource};

use crate::audio::AudioOutput;
use crate::decoders::ils_loc::{
    ILS_MORSE_AUDIO_BPF_HIGH, ILS_MORSE_AUDIO_BPF_LOW, ILS_MORSE_AUDIO_BPF_ORDER, IlsFrame,
    IlsLocalizerDemodulator, IlsMorseCandidate, IlsMorseDebugInfo, compute_ddm,
};
use crate::decoders::metrics;
use crate::device_uri::{build_device_source, is_device_uri};
use crate::timestamp_helpers::{resolve_file_start_unix, unix_now_seconds};
use desperado::dsp::filters::ButterworthFilter;

const DEFAULT_CHUNK_SAMPLES: usize = 262_144;

enum TimestampBase {
    FileStartUnix(f64),
    LiveWallClock,
}

/// Iterator that reads raw I/Q data and emits decoded [`IlsFrame`] measurements.
pub struct IlsLocalizerSource {
    source: BaseIqSource,
    demodulator: IlsLocalizerDemodulator,
    ils_frequency: f64,
    center_frequency: f64,
    sample_count: usize,
    /// Samples per 1-second DDM window at audio rate.
    window_samples: usize,
    /// Samples in the Morse decode accumulator (15 seconds).
    morse_window_samples: usize,
    // Accumulation buffers (audio rate)
    env_90_buf: Vec<f64>,
    env_150_buf: Vec<f64>,
    audio_buf: Vec<f64>,
    morse_audio_buf: Vec<f64>,
    timestamp_base: TimestampBase,
    /// Most-recently decoded ident (persists across windows).
    current_ident: Option<String>,
    debug_morse: bool,
    all_tokens_history: Vec<String>,
    morse_bpf: ButterworthFilter,
    audio_output: Option<AudioOutput>,
}

impl IlsLocalizerSource {
    /// Open an ILS source from a file path or device URI.
    ///
    /// # Parameters
    /// - `path`: path to a gqrx `.raw` file **or** a device URI
    ///   (`rtlsdr://`, `soapy://`, `airspy://`)
    /// - `sample_rate`: I/Q sample rate in Hz (e.g. `1_800_000`)
    /// - `format`: I/Q sample format (use [`IqFormat::Cf32`] for gqrx files)
    /// - `ils_freq_mhz`: ILS localizer frequency in MHz (e.g. `110.695`)
    /// - `center_freq_mhz`: SDR centre frequency in MHz (e.g. `110.7`)
    /// - `window_seconds`: DDM integration window length (default: `1.0`)
    /// - `morse_window_seconds`: Morse accumulation window (default: `15.0`)
    #[allow(clippy::too_many_arguments)]
    pub fn new<P: AsRef<Path>>(
        path: P,
        sample_rate: u32,
        format: IqFormat,
        ils_freq_mhz: f64,
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

        let demodulator = IlsLocalizerDemodulator::new(sample_rate);
        let audio_rate = demodulator.audio_rate();
        let window_samples = (window_seconds * audio_rate).round() as usize;
        let morse_window_samples = (morse_window_seconds * audio_rate).round() as usize;

        let morse_bpf = ButterworthFilter::bandpass(
            ILS_MORSE_AUDIO_BPF_LOW,
            ILS_MORSE_AUDIO_BPF_HIGH,
            audio_rate,
            ILS_MORSE_AUDIO_BPF_ORDER,
        );

        Ok(Self {
            source,
            demodulator,
            ils_frequency: ils_freq_mhz * 1e6,
            center_frequency: center_freq_mhz * 1e6,
            sample_count: 0,
            window_samples,
            morse_window_samples,
            env_90_buf: Vec::new(),
            env_150_buf: Vec::new(),
            audio_buf: Vec::new(),
            morse_audio_buf: Vec::new(),
            timestamp_base,
            current_ident: None,
            debug_morse,
            all_tokens_history: Vec::new(),
            morse_bpf,
            audio_output: None,
        })
    }

    /// Set audio output for morse ident playback.
    pub fn set_audio_output(&mut self, audio: AudioOutput) {
        self.audio_output = Some(audio);
    }
}

impl Iterator for IlsLocalizerSource {
    type Item = Result<IlsFrame, io::Error>;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            let samples = match self.source.next() {
                Some(Ok(s)) if s.is_empty() => return None,
                Some(Ok(s)) => s,
                Some(Err(e)) => return Some(Err(io::Error::other(e.to_string()))),
                None => return None,
            };

            let freq_offset = self.ils_frequency - self.center_frequency;
            self.sample_count += samples.len();

            let (env_90, env_150, audio) = self.demodulator.demodulate(&samples, freq_offset);

            self.env_90_buf.extend(&env_90);
            self.env_150_buf.extend(&env_150);
            self.audio_buf.extend(&audio);
            self.morse_audio_buf.extend(&audio);

            // Stream filtered audio samples to soundcard if audio output is enabled
            if let Some(ref audio_out) = self.audio_output {
                let filtered_audio = self.morse_bpf.filter(&audio);
                // Normalize to f32 range [-1.0, 1.0] and send to audio output
                // Normalize based on typical signal envelope (mean ~0.5)
                for sample in filtered_audio {
                    let normalized = (sample / 0.5).clamp(-1.0, 1.0) as f32;
                    let _ = audio_out.send(normalized);
                }
            }

            if self.audio_buf.len() >= self.window_samples {
                // Compute DDM for this window
                let ddm_result = compute_ddm(
                    &self.env_90_buf[..self.window_samples.min(self.env_90_buf.len())],
                    &self.env_150_buf[..self.window_samples.min(self.env_150_buf.len())],
                    &self.audio_buf[..self.window_samples],
                );

                let ddm_data = match ddm_result {
                    Ok(result) => (
                        result.ddm,
                        result.mod_90_hz,
                        result.mod_150_hz,
                        result.carrier_strength,
                    ),
                    Err(_) => (0.0, 0.0, 0.0, 0.0),
                };
                let (ddm, mod_90_pct, mod_150_pct, signal_strength) = ddm_data;

                // Try Morse decode if accumulator is large enough
                let mut morse_debug = None;
                if self.morse_audio_buf.len() >= self.morse_window_samples {
                    let (ident_single, tokens, attempts) =
                        self.demodulator.decode_ident(&self.morse_audio_buf);
                    self.all_tokens_history.extend(tokens);

                    // Use the decoded ident if available, otherwise keep current
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
                        let mut candidates: Vec<IlsMorseCandidate> = counts
                            .into_iter()
                            .map(|(token, count)| IlsMorseCandidate {
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

                        morse_debug = Some(IlsMorseDebugInfo {
                            candidates,
                            total_tokens: self.all_tokens_history.len(),
                            decode_attempts: attempts,
                        });
                    }

                    // Slide the Morse window forward by half its length
                    let keep = self.morse_window_samples / 2;
                    if self.morse_audio_buf.len() > self.morse_window_samples + keep {
                        self.morse_audio_buf.drain(0..keep);
                    }
                }

                let elapsed = self.sample_count as f64 / self.demodulator.sample_rate;
                let timestamp = match self.timestamp_base {
                    TimestampBase::FileStartUnix(t0) => t0 + elapsed,
                    TimestampBase::LiveWallClock => unix_now_seconds(),
                };

                let frame = IlsFrame::new(
                    metrics::round_decimals(timestamp, 5),
                    self.ils_frequency / 1e6,
                    metrics::round_decimals(ddm, 4),
                    metrics::round_decimals(mod_90_pct, 2),
                    metrics::round_decimals(mod_150_pct, 2),
                    metrics::round_decimals(signal_strength, 4),
                    self.current_ident.clone(),
                )
                .with_morse_debug(morse_debug);

                // Consume the processed window, keeping any overflow
                self.audio_buf
                    .drain(0..self.window_samples.min(self.audio_buf.len()));
                self.env_90_buf
                    .drain(0..self.window_samples.min(self.env_90_buf.len()));
                self.env_150_buf
                    .drain(0..self.window_samples.min(self.env_150_buf.len()));

                return Some(Ok(frame));
            }
        }
    }
}
