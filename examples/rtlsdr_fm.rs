//! FM Demodulator for various IQ sources (mono or stereo with RDS)
//!
//! This example demonstrates FM demodulation with support for:
//! - RTL-SDR devices
//! - SoapySDR-compatible devices
//! - IQ file playback
//! - Mono or stereo decoding
//! - RDS (Radio Data System) decoding
//!
//! # Usage Examples
//!
//! ## RTL-SDR (mono)
//! ```bash
//! cargo run --example rtlsdr_fm --features "rtlsdr audio clap" -- \
//!     -c 105.1M --source rtlsdr
//! ```
//!
//! ## RTL-SDR (stereo with RDS)
//! ```bash
//! cargo run --example rtlsdr_fm --features "rtlsdr audio clap" -- \
//!     -c 105.1M --source rtlsdr --stereo -v
//! ```
//!
//! ## SoapySDR
//! ```bash
//! cargo run --example rtlsdr_fm --features "soapy audio clap" -- \
//!     -c 105.1M --source soapy --soapy-args "driver=hackrf"
//! ```
//!
//! ## IQ File Playback
//! ```bash
//! cargo run --example rtlsdr_fm --features "audio clap" -- \
//!     -c 105.1M --source file --file samples.iq --format cu8
//! ```

use crossbeam::channel;
use desperado::dsp::{
    DspBlock,
    afc::SquareFreqOffsetCorrection,
    decimator::Decimator,
    filters::LowPassFir,
    fm::{DeemphasisFilter, PhaseExtractor},
    rds::RdsParser,
    rotate::Rotate,
};
use futures::StreamExt;
use std::f32::consts::PI;
use std::io::{Write, stdout};
use std::str::FromStr;

use clap::{Parser, ValueEnum};
use desperado::{IqAsyncSource, IqFormat};

use rubato::{
    Resampler, SincFixedOut, SincInterpolationParameters, SincInterpolationType, WindowFunction,
};
use tinyaudio::prelude::*;

#[derive(Debug, Clone, Copy)]
struct Frequency(u32);

#[derive(Debug, Clone, Copy, ValueEnum)]
enum SourceType {
    Rtlsdr,
    Soapy,
    File,
}

#[derive(Parser, Debug)]
#[command(author, version, about = "FM demodulator from various IQ sources (mono or stereo)", long_about = None)]
struct Args {
    /// Center frequency in Hz (accepts k/M suffix, e.g. 105.1M)
    #[arg(short, long, value_parser = Frequency::from_str)]
    center_freq: Frequency,

    /// Sample rate in Hz
    #[arg(short, long, default_value_t = 2_000_000)]
    sample_rate: u32,

    /// Tuner gain (None for AGC, Some(gain) for manual)
    #[arg(short, long, default_value = None)]
    gain: Option<i32>,

    /// Frequency offset in Hz (can be negative)
    #[arg(short, long, default_value_t = 200_000, allow_hyphen_values = true)]
    offset_freq: i32,

    /// Enable automatic frequency correction
    #[arg(short, long, default_value_t = false)]
    afc: bool,

    /// Enable stereo decoding (with RDS support)
    #[arg(long, default_value_t = false)]
    stereo: bool,

    /// Enable verbose RDS output
    #[arg(short, long, default_value_t = false)]
    verbose: bool,

    /// Disable audio output (for SSH/headless operation)
    #[arg(long, default_value_t = false)]
    no_audio: bool,

    /// Source type: rtlsdr, soapy, or file
    #[arg(long, default_value = "rtlsdr")]
    source: SourceType,

    /// Input file path (required when source=file)
    #[arg(long)]
    file: Option<String>,

    /// IQ format for file input (cu8, cs8, cs16, cf32)
    #[arg(long, default_value = "cu8")]
    format: String,

    /// SoapySDR device arguments (e.g., "driver=rtlsdr")
    #[arg(long, default_value = "driver=rtlsdr")]
    soapy_args: String,

    /// SoapySDR channel
    #[arg(long, default_value_t = 0)]
    soapy_channel: usize,

    /// RTL-SDR device index
    #[arg(long, default_value_t = 0)]
    device_index: usize,
}

const FM_BANDWIDTH: f32 = 240_000.0;
const MONO_SIGNAL_BW: f32 = 15_000.0;
const AUDIO_RATE: usize = 48_000;

#[tokio::main]
async fn main() -> desperado::Result<()> {
    let args = Args::parse();

    // Create IQ source based on selected type
    // Calculate tuning frequency: center_freq - offset_freq (handles negative offset)
    let tuning_freq = if args.offset_freq >= 0 {
        args.center_freq.0 - args.offset_freq as u32
    } else {
        args.center_freq.0 + (-args.offset_freq) as u32
    };

    let mut iq_source = match args.source {
        SourceType::Rtlsdr => {
            #[cfg(feature = "rtlsdr")]
            {
                IqAsyncSource::from_rtlsdr(
                    args.device_index,
                    tuning_freq,
                    args.sample_rate,
                    args.gain,
                )
                .await?
            }
            #[cfg(not(feature = "rtlsdr"))]
            {
                eprintln!("Error: rtlsdr feature not enabled. Rebuild with --features rtlsdr");
                std::process::exit(1);
            }
        }
        SourceType::Soapy => {
            #[cfg(feature = "soapy")]
            {
                IqAsyncSource::from_soapysdr(
                    &args.soapy_args,
                    args.soapy_channel,
                    tuning_freq,
                    args.sample_rate,
                    args.gain.map(|g| g as f64),
                    "TUNER",
                )
                .await?
            }
            #[cfg(not(feature = "soapy"))]
            {
                eprintln!("Error: soapy feature not enabled. Rebuild with --features soapy");
                std::process::exit(1);
            }
        }
        SourceType::File => {
            let file_path = args
                .file
                .as_ref()
                .expect("--file is required when source=file");
            let format = IqFormat::from_str(&args.format)
                .map_err(|e| std::io::Error::other(format!("Invalid format: {}", e)))?;

            IqAsyncSource::from_file(
                file_path,
                tuning_freq,
                args.sample_rate,
                16384, // chunk size
                format,
            )
            .await?
        }
    };

    println!(
        "FM demodulator: {} mode, source: {:?}",
        if args.stereo { "stereo" } else { "mono" },
        args.source
    );

    let channels = if args.stereo { 2 } else { 1 };

    // Setup audio output - larger buffer for file playback to prevent underruns
    let is_file_source = matches!(args.source, SourceType::File);
    let buffer_size = if is_file_source {
        AUDIO_RATE * 4 // 4 seconds for file playback
    } else {
        AUDIO_RATE * 2 // 2 seconds for live sources
    };
    let (tx, rx) = channel::bounded::<f32>(buffer_size);

    let _device = if !args.no_audio {
        let config = OutputDeviceParameters {
            channels_count: channels,
            sample_rate: AUDIO_RATE,
            channel_sample_count: 1024,
        };
        Some(
            run_output_device(config, move |data| {
                for sample in data.iter_mut() {
                    *sample = rx.try_recv().unwrap_or(0.0);
                }
            })
            .unwrap(),
        )
    } else {
        println!("Audio output disabled (--no-audio)");
        None
    };

    let mut rotate = Rotate::new(-2.0 * PI * args.offset_freq as f32 / args.sample_rate as f32);
    let mut phase_extractor = PhaseExtractor::new();
    let factor = (args.sample_rate as f32 / FM_BANDWIDTH).round() as usize;
    let mut decimator = Decimator::new(factor);

    let n = 2048;
    let window = 64;
    let mut afc = SquareFreqOffsetCorrection::with_params(n, window, false);

    let lowpass_fir = LowPassFir::new(MONO_SIGNAL_BW, FM_BANDWIDTH, 256);

    if args.stereo {
        println!("Running stereo FM demodulator with RDS...");
        run_stereo(
            &mut iq_source,
            &args,
            &mut rotate,
            &mut phase_extractor,
            &mut decimator,
            &mut afc,
            &lowpass_fir,
            tx,
        )
        .await?;
    } else {
        println!("Running mono FM demodulator...");
        run_mono(
            &mut iq_source,
            &args,
            &mut rotate,
            &mut phase_extractor,
            &mut decimator,
            &mut afc,
            &lowpass_fir,
            tx,
        )
        .await?;
    }

    Ok(())
}

#[allow(clippy::too_many_arguments)]
async fn run_mono(
    iq_source: &mut IqAsyncSource,
    args: &Args,
    rotate: &mut Rotate,
    phase_extractor: &mut PhaseExtractor,
    decimator: &mut Decimator,
    afc: &mut SquareFreqOffsetCorrection,
    lowpass_fir: &LowPassFir,
    tx: channel::Sender<f32>,
) -> desperado::Result<()> {
    let mut deemphasis = DeemphasisFilter::new(FM_BANDWIDTH, 50e-6);
    let mut audio_resample =
        AudioAdaptiveResampler::new(AUDIO_RATE as f64 / FM_BANDWIDTH as f64, 1, 1);

    let mut agc_gain = 0.5f32;
    const AGC_ATTACK: f32 = 0.999;
    const AGC_RELEASE: f32 = 0.9999;

    // Real-time pacing for file playback - track absolute time to prevent drift
    let is_file_source = matches!(args.source, SourceType::File);

    while let Some(chunk) = iq_source.next().await {
        let chunk = chunk.map_err(|e| std::io::Error::other(format!("{}", e)))?;

        let shifted = rotate.process(&chunk);
        let decimated = decimator.process(&shifted);
        let afc_corrected = if args.afc {
            afc.process(&decimated)
        } else {
            decimated
        };
        let phase = phase_extractor.process(&afc_corrected);
        let filtered = lowpass_fir.process(&phase);
        let deemphasized = deemphasis.process(&filtered);

        let audio = audio_resample.process(&deemphasized);

        let chunk_max = audio.iter().fold(0.0_f32, |a, &b| a.max(b.abs()));

        if chunk_max > 0.0001 {
            let target_gain = 0.5 / chunk_max;
            if target_gain < agc_gain {
                agc_gain = agc_gain * AGC_ATTACK + target_gain * (1.0 - AGC_ATTACK);
            } else {
                agc_gain = agc_gain * AGC_RELEASE + target_gain * (1.0 - AGC_RELEASE);
            }
            agc_gain = agc_gain.clamp(0.01, 2.0);
        }

        let processed: Vec<f32> = audio
            .iter()
            .map(|&s| {
                let scaled = s * agc_gain;
                if scaled > 1.0 {
                    1.0 - (-10.0 * (scaled - 1.0)).exp()
                } else if scaled < -1.0 {
                    -1.0 + (-10.0 * (-scaled - 1.0)).exp()
                } else {
                    scaled
                }
            })
            .collect();

        if !args.no_audio {
            // Only do adaptive resampling for live sources (not files)
            if !is_file_source {
                audio_resample.adjust_ratio(tx.len() as f64 / (AUDIO_RATE * 2) as f64);
            }

            // For file sources, use blocking send to let audio buffer control pacing
            // For live sources, use non-blocking to avoid dropping samples from hardware
            if is_file_source {
                // Blocking send - wait for buffer space (natural pacing)
                for sample in processed {
                    if tx.send(sample).is_err() {
                        break;
                    }
                }
            } else {
                // Non-blocking for live sources
                if tx.len() > (AUDIO_RATE * 2) * 9 / 10 {
                    println!(
                        "\n[DROP] Buffer >90% full, dropping {} samples",
                        processed.len()
                    );
                    continue;
                }

                for sample in processed {
                    if tx.try_send(sample).is_err() {
                        println!("\n[WARNING] Audio buffer full");
                        break;
                    }
                }
            }
        }
    }

    Ok(())
}

#[allow(clippy::too_many_arguments)]
async fn run_stereo(
    iq_source: &mut IqAsyncSource,
    args: &Args,
    rotate: &mut Rotate,
    phase_extractor: &mut PhaseExtractor,
    decimator: &mut Decimator,
    afc: &mut SquareFreqOffsetCorrection,
    lowpass_fir: &LowPassFir,
    tx: channel::Sender<f32>,
) -> desperado::Result<()> {
    let mut stereo = StereoDecoderPLL::new(FM_BANDWIDTH);
    let mut deemphasis_l = DeemphasisFilter::new(FM_BANDWIDTH, 50e-6);
    let mut deemphasis_r = DeemphasisFilter::new(FM_BANDWIDTH, 50e-6);
    let mut rds = RdsDecoder::new(FM_BANDWIDTH, args.verbose);

    let mut audio_resample =
        AudioAdaptiveResampler::new(AUDIO_RATE as f64 / FM_BANDWIDTH as f64, 5, 2);

    // Real-time pacing for file playback - track absolute time to prevent drift
    let is_file_source = matches!(args.source, SourceType::File);

    while let Some(chunk) = iq_source.next().await {
        let chunk = chunk.map_err(|e| std::io::Error::other(format!("{}", e)))?;

        let shifted = rotate.process(&chunk);
        let decimated = decimator.process(&shifted);
        let afc_corrected = if args.afc {
            afc.process(&decimated)
        } else {
            decimated
        };
        let phase = phase_extractor.process(&afc_corrected);
        let filtered = lowpass_fir.process(&phase);

        let (left, right) = stereo.process(&filtered);
        let deem_l = deemphasis_l.process(&left);
        let deem_r = deemphasis_r.process(&right);

        rds.process(&filtered);

        // Interleave stereo
        let mut interleaved = Vec::with_capacity(deem_l.len() * 2);
        for i in 0..deem_l.len() {
            interleaved.push(deem_l[i]);
            interleaved.push(deem_r[i]);
        }

        let audio = audio_resample.process(&interleaved);

        if !args.no_audio {
            // Only do adaptive resampling for live sources (not files)
            if !is_file_source {
                audio_resample.adjust_ratio(tx.len() as f64 / (AUDIO_RATE * 2) as f64);
            }

            // For file sources, use blocking send to let audio buffer control pacing
            // For live sources, use non-blocking to avoid dropouts
            if is_file_source {
                for sample in audio {
                    if tx.send(sample).is_err() {
                        break;
                    }
                }
            } else {
                for sample in audio {
                    if tx.try_send(sample).is_err() {
                        break;
                    }
                }
            }
        }
    }

    Ok(())
}

impl FromStr for Frequency {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let s = s.trim();
        if let Some(stripped) = s.strip_suffix('M') {
            let val: f32 = stripped.trim().parse().map_err(|_| "Invalid MHz value")?;
            Ok(Frequency((val * 1_000_000.0) as u32))
        } else if let Some(stripped) = s.strip_suffix('k') {
            let val: f32 = stripped.trim().parse().map_err(|_| "Invalid kHz value")?;
            Ok(Frequency((val * 1_000.0) as u32))
        } else {
            let val: u32 = s.parse().map_err(|_| "Invalid Hz value")?;
            Ok(Frequency(val))
        }
    }
}

struct AudioAdaptiveResampler {
    resampler: SincFixedOut<f32>,
    target_fill: f64,
    alpha: f64,
    k_p: f64,
    k_i: f64,
    smoothed_error: f64,
    integral_error: f64,
    adjustment_interval: usize,
    adjustment_counter: usize,
    leftover: Vec<f32>,
    resample_ratio: f64,
    channels: usize,
}

impl AudioAdaptiveResampler {
    fn new(initial_ratio: f64, adjustment_interval: usize, channels: usize) -> Self {
        let params = SincInterpolationParameters {
            sinc_len: 256,
            f_cutoff: 0.95,
            interpolation: SincInterpolationType::Cubic,
            oversampling_factor: 160,
            window: WindowFunction::BlackmanHarris2,
        };

        let output_frames = 1024;

        let resampler =
            SincFixedOut::<f32>::new(initial_ratio, 2.0, params, output_frames, channels).unwrap();

        Self {
            resampler,
            target_fill: if channels == 1 { 0.3 } else { 0.4 },
            alpha: if channels == 1 { 0.95 } else { 0.9 },
            k_p: if channels == 1 { 0.0001 } else { 0.002 },
            k_i: if channels == 1 { 1e-7 } else { 5e-6 },
            smoothed_error: 0.0,
            integral_error: 0.0,
            adjustment_interval,
            adjustment_counter: 0,
            leftover: Vec::new(),
            resample_ratio: initial_ratio,
            channels,
        }
    }

    fn adjust_ratio(&mut self, buffer_fill: f64) {
        self.adjustment_counter += 1;
        if self.adjustment_counter >= self.adjustment_interval {
            self.adjustment_counter = 0;

            let error = self.target_fill - buffer_fill;

            self.smoothed_error = self.alpha * self.smoothed_error + (1.0 - self.alpha) * error;

            if self.smoothed_error.abs() < 0.01 {
                self.smoothed_error = 0.0;
            }

            if self.smoothed_error.abs() < 0.15 {
                self.integral_error += self.smoothed_error;
                self.integral_error = self.integral_error.clamp(-100.0, 100.0);
            }

            self.resample_ratio += self.k_p * self.smoothed_error + self.k_i * self.integral_error;

            let nominal = AUDIO_RATE as f64 / FM_BANDWIDTH as f64;
            let min_ratio = nominal * 0.97;
            let max_ratio = nominal * 1.02;

            self.resample_ratio = self.resample_ratio.clamp(min_ratio, max_ratio);

            self.resampler
                .set_resample_ratio(self.resample_ratio, true)
                .unwrap();

            if self.channels == 1 {
                print!(
                    "\rBuf: {:.1}% | Ratio: {:.6} (nom: {:.6}) | Err: {:.3} | smooth: {:.3} | integral: {:.3}      ",
                    buffer_fill * 100.0,
                    self.resample_ratio,
                    nominal,
                    error,
                    self.smoothed_error,
                    self.integral_error
                );
                stdout().flush().unwrap();
            }
        }
    }

    fn process(&mut self, input: &[f32]) -> Vec<f32> {
        self.leftover.extend_from_slice(input);

        let mut output = Vec::new();

        loop {
            let input_frames_needed = self.resampler.input_frames_next();
            let samples_needed = input_frames_needed * self.channels;

            if self.leftover.len() < samples_needed {
                break;
            }

            let chunk: Vec<f32> = self.leftover.drain(..samples_needed).collect();

            // For mono: simple vector wrapping
            // For stereo: deinterleave
            let input_block = if self.channels == 1 {
                vec![chunk]
            } else {
                let mut chs: Vec<Vec<f32>> =
                    vec![Vec::with_capacity(input_frames_needed); self.channels];
                for frame_idx in 0..input_frames_needed {
                    #[allow(clippy::needless_range_loop)]
                    for ch in 0..self.channels {
                        let idx = frame_idx * self.channels + ch;
                        chs[ch].push(chunk[idx]);
                    }
                }
                chs
            };

            match self.resampler.process(&input_block, None) {
                Ok(output_block) => {
                    if self.channels == 1 {
                        output.extend_from_slice(&output_block[0]);
                    } else {
                        // Re-interleave stereo
                        let out_frames = output_block[0].len();
                        #[allow(clippy::needless_range_loop)]
                        for i in 0..out_frames {
                            for ch in 0..self.channels {
                                output.push(output_block[ch][i]);
                            }
                        }
                    }
                }
                Err(e) => {
                    eprintln!("Resampler error: {:?}", e);
                    break;
                }
            }
        }

        output
    }
}

/// Stereo decoder using a complex PLL locked to the 19 kHz pilot tone.
pub struct StereoDecoderPLL {
    sample_rate: f32,
    pll_phase: f64,
    pll_freq: f64,
    nominal_freq: f64,
    kp: f64,
    ki: f64,
    pilot_hi: LowPassFir,
    pilot_lo: LowPassFir,
    diff_lowpass: LowPassFir,
}

impl StereoDecoderPLL {
    pub fn new(sample_rate: f32) -> Self {
        let nominal = 19_000.0_f64;
        Self {
            sample_rate,
            pll_phase: 0.0,
            pll_freq: nominal,
            nominal_freq: nominal,
            kp: 0.02,
            ki: 1e-4,
            pilot_hi: LowPassFir::new(19_700.0, sample_rate, 257),
            pilot_lo: LowPassFir::new(18_300.0, sample_rate, 257),
            diff_lowpass: LowPassFir::new(15_000.0, sample_rate, 257),
        }
    }

    pub fn process(&mut self, input: &[f32]) -> (Vec<f32>, Vec<f32>) {
        let n = input.len();
        let pilot_hi = self.pilot_hi.process(input);
        let pilot_lo = self.pilot_lo.process(input);
        let mut pilot_band = Vec::with_capacity(n);
        for i in 0..n {
            pilot_band.push(pilot_hi[i] - pilot_lo[i]);
        }

        let mut lmr_raw = Vec::with_capacity(n);
        for &p in &pilot_band {
            let p = p as f64;
            let cos_p = (self.pll_phase).cos();
            let sin_p = (self.pll_phase).sin();

            let i_mix = p * cos_p;
            let q_mix = p * sin_p;
            let error = q_mix.atan2(i_mix);

            self.pll_freq += self.ki * error;
            self.pll_freq = self
                .pll_freq
                .clamp(self.nominal_freq * 0.95, self.nominal_freq * 1.05);
            self.pll_phase += 2.0 * std::f64::consts::PI * self.pll_freq / self.sample_rate as f64
                + self.kp * error;
            if self.pll_phase > std::f64::consts::PI {
                self.pll_phase -= 2.0 * std::f64::consts::PI;
            } else if self.pll_phase < -std::f64::consts::PI {
                self.pll_phase += 2.0 * std::f64::consts::PI;
            }

            let carrier_38 = (2.0 * self.pll_phase).sin();
            lmr_raw.push(input[lmr_raw.len()] as f64 * carrier_38);
        }

        let lmr_f32: Vec<f32> = lmr_raw.iter().map(|&v| v as f32).collect();
        let lmr = self.diff_lowpass.process(&lmr_f32);

        let mut left = Vec::with_capacity(n);
        let mut right = Vec::with_capacity(n);
        for i in 0..n {
            let lpr = input[i];
            let lmr_val = lmr[i];
            left.push(0.5 * (lpr + lmr_val));
            right.push(0.5 * (lpr - lmr_val));
        }
        (left, right)
    }
}

/// RDS decoder with Costas loop and symbol timing recovery
pub struct RdsDecoder {
    sample_rate: f32,
    pll_phase: f64,
    pll_freq: f64,
    nominal_freq: f64,
    pll_alpha: f64,
    pll_beta: f64,
    bpf_hi: LowPassFir,
    bpf_lo: LowPassFir,
    lpf: LowPassFir,
    #[allow(dead_code)]
    symbol_rate: f32,
    symbol_phase: f32,
    symbol_period: f32,
    ted_gain: f32,
    last_bit: u8,
    bits: Vec<u8>,
    rds_parser: RdsParser,
    last_station: Option<String>,
    last_radiotext: Option<String>,
}

impl RdsDecoder {
    pub fn new(sample_rate: f32, verbose: bool) -> Self {
        let nominal = 57_000.0_f64;
        let mut rds_parser = RdsParser::new();
        rds_parser.set_verbose(verbose);

        Self {
            sample_rate,
            pll_phase: 0.0,
            pll_freq: nominal,
            nominal_freq: nominal,
            pll_alpha: 0.01,
            pll_beta: 0.00001,
            bpf_hi: LowPassFir::new(60_000.0, sample_rate, 257),
            bpf_lo: LowPassFir::new(54_000.0, sample_rate, 257),
            lpf: LowPassFir::new(2_400.0, sample_rate, 257),
            symbol_rate: 1187.5,
            symbol_phase: 0.0,
            symbol_period: sample_rate / 1187.5,
            ted_gain: 0.01,
            last_bit: 0,
            bits: Vec::new(),
            rds_parser,
            last_station: None,
            last_radiotext: None,
        }
    }

    pub fn process(&mut self, input: &[f32]) {
        // Bandpass filter around 57 kHz
        let hi = self.bpf_hi.process(input);
        let lo = self.bpf_lo.process(input);
        let mut bandpass = Vec::with_capacity(input.len());
        for i in 0..input.len() {
            bandpass.push(hi[i] - lo[i]);
        }

        // Costas loop for carrier recovery (BPSK)
        let mut demod = Vec::with_capacity(bandpass.len());
        for &x in &bandpass {
            let cos_phase = (2.0 * std::f64::consts::PI * self.pll_phase).cos() as f32;
            let sin_phase = (2.0 * std::f64::consts::PI * self.pll_phase).sin() as f32;

            let i_mix = x * cos_phase;
            let q_mix = x * sin_phase;

            demod.push(i_mix);

            let error = i_mix.signum() * q_mix;

            self.pll_freq += self.pll_beta * error as f64;
            self.pll_freq = self
                .pll_freq
                .clamp(self.nominal_freq * 0.98, self.nominal_freq * 1.02);

            self.pll_phase +=
                self.pll_freq / self.sample_rate as f64 + self.pll_alpha * error as f64;

            while self.pll_phase >= 1.0 {
                self.pll_phase -= 1.0;
            }
            while self.pll_phase < 0.0 {
                self.pll_phase += 1.0;
            }
        }

        // Lowpass filter
        let filtered = self.lpf.process(&demod);

        // Symbol timing recovery with Gardner TED
        let mut symbols = Vec::new();
        let mut i = 0;
        while i < filtered.len() {
            let sample_idx = self.symbol_phase as usize;
            if sample_idx + 2 < filtered.len() {
                let mid = filtered[sample_idx + 1];
                symbols.push(mid);

                if sample_idx > 0 && sample_idx + 2 < filtered.len() {
                    let early = filtered[sample_idx];
                    let late = filtered[sample_idx + 2];
                    let error = (late - early) * mid;

                    self.symbol_phase += self.symbol_period - self.ted_gain * error;
                } else {
                    self.symbol_phase += self.symbol_period;
                }
            } else {
                break;
            }

            i = self.symbol_phase as usize;
        }

        self.symbol_phase -= filtered.len() as f32;
        if self.symbol_phase < 0.0 {
            self.symbol_phase += self.symbol_period;
        }

        // Differential Manchester decoding
        for &symbol in &symbols {
            let bit = if symbol > 0.0 { 1 } else { 0 };
            let decoded = bit ^ self.last_bit;
            self.bits.push(decoded);
            self.last_bit = bit;
        }

        // Feed to RDS parser
        if self.bits.len() >= 104 {
            self.rds_parser.push_bits(&self.bits);

            if let Some(station) = self.rds_parser.station_name()
                && self.last_station.as_ref() != Some(&station)
            {
                println!("[RDS] Station: {}", station);
                self.last_station = Some(station);
            }
            if let Some(text) = self.rds_parser.radio_text()
                && self.last_radiotext.as_ref() != Some(&text)
            {
                println!("[RDS] Radio Text: {}", text);
                self.last_radiotext = Some(text);
            }

            self.bits.clear();
        }
    }
}
