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

use std::io;
use std::path::Path;

use desperado::{IqFormat, IqSource as BaseIqSource};

use crate::audio::AudioOutput;
use crate::decoders::{
    VOR_MORSE_AUDIO_BPF_HIGH, VOR_MORSE_AUDIO_BPF_LOW, VOR_MORSE_AUDIO_BPF_ORDER, VorDemodulator,
    VorProcessor, VorRadial, metrics,
};
use crate::device_uri::{build_device_source, is_device_uri};
use crate::timestamp_helpers::{resolve_file_start_unix, unix_now_seconds};
use desperado::dsp::filters::ButterworthFilter;

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
    window_iq_count: usize,
    window_clip_count: usize,
    timestamp_base: TimestampBase,
    audio_output: Option<AudioOutput>,
    morse_bpf: ButterworthFilter,
    processor: VorProcessor,
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

        let morse_bpf = ButterworthFilter::bandpass(
            VOR_MORSE_AUDIO_BPF_LOW,
            VOR_MORSE_AUDIO_BPF_HIGH,
            audio_rate,
            VOR_MORSE_AUDIO_BPF_ORDER,
        );

        let processor = VorProcessor::new(
            window_seconds,
            morse_window_seconds,
            audio_rate,
            debug_morse,
        );

        Ok(Self {
            source,
            demodulator,
            vor_frequency: vor_freq_mhz * 1e6,
            center_frequency: center_freq_mhz * 1e6,
            sample_count: 0,
            window_iq_count: 0,
            window_clip_count: 0,
            timestamp_base,
            audio_output: None,
            morse_bpf,
            processor,
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

            // Count I/Q clipping for signal quality metrics
            let freq_offset = self.vor_frequency - self.center_frequency;
            for sample in &samples {
                self.window_iq_count += 1;
                if sample.re.abs() >= 0.98 || sample.im.abs() >= 0.98 {
                    self.window_clip_count += 1;
                }
            }

            // Demodulate I/Q to baseband signals
            let (var_30, ref_30, audio) = self.demodulator.demodulate(&samples, freq_offset);

            // Stream audio to output if enabled
            if let Some(ref audio_out) = self.audio_output {
                let filtered_audio = self.morse_bpf.filter(&audio);
                for sample in filtered_audio {
                    let normalized = (sample / 0.5).clamp(-1.0, 1.0) as f32;
                    let _ = audio_out.send(normalized);
                }
            }

            self.sample_count += samples.len();

            // Accumulate signals into processor
            if self.processor.accumulate(&audio, &var_30, &ref_30) {
                // Window is ready - emit radial measurement
                let audio_rate = self.demodulator.audio_rate();
                let clipping_ratio = if self.window_iq_count > 0 {
                    Some(self.window_clip_count as f64 / self.window_iq_count as f64)
                } else {
                    None
                };

                let elapsed = self.sample_count as f64 / self.demodulator.sample_rate;
                let timestamp = match self.timestamp_base {
                    TimestampBase::FileStartUnix(t0) => t0 + elapsed,
                    TimestampBase::LiveWallClock => unix_now_seconds(),
                };

                if let Some(output) =
                    self.processor
                        .emit(timestamp, clipping_ratio, self.window_iq_count, audio_rate)
                {
                    self.window_iq_count = 0;
                    self.window_clip_count = 0;

                    let vor_radial = VorRadial::new(
                        metrics::round_decimals(timestamp, 5),
                        metrics::round_decimals(output.radial, 2),
                        self.vor_frequency / 1e6,
                    )
                    .with_quality(output.signal_quality)
                    .with_ident(output.ident)
                    .with_morse_debug(output.morse_debug);

                    return Some(Ok(vor_radial));
                }
            }
        }
    }
}
