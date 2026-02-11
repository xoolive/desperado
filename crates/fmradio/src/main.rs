//! FM Demodulator for various IQ sources (mono or stereo with RDS)
//!
//! This application demonstrates FM demodulation with support for:
//! - RTL-SDR devices
//! - Airspy devices
//! - SoapySDR-compatible devices
//! - IQ file playback
//! - Mono or stereo decoding
//! - RDS (Radio Data System) decoding
//!
//! # Usage Examples
//!
//! ## RTL-SDR (mono)
//! ```bash
//! fmradio -c 105.1M --source rtlsdr
//! ```
//!
//! ## Airspy (mono)
//! ```bash
//! fmradio -c 105.1M --source airspy
//! ```
//!
//! ## RTL-SDR (stereo with RDS)
//! ```bash
//! fmradio -c 105.1M --source rtlsdr --stereo -v
//! ```
//!
//! ## SoapySDR
//! ```bash
//! fmradio -c 105.1M --source soapy --soapy-args "driver=hackrf"
//! ```
//!
//! ## IQ File Playback
//! ```bash
//! fmradio -c 105.1M --source file --file samples.iq --format cu8
//! ```

use crossbeam::channel;
use desperado::dsp::{
    DspBlock,
    afc::SquareFreqOffsetCorrection,
    agc::Agc,
    decimator::Decimator,
    filters::LowPassFir,
    nco::Nco,
    rotate::Rotate,
    symsync::SymSync,
};
use fmradio::fm::{DeemphasisFilter, PhaseExtractor};
use fmradio::rds::{RdsParser, DIFlags};
use futures::StreamExt;
use std::f32::consts::PI;
use std::io::{Write, stdout};
use std::str::FromStr;
use tracing::{info, debug, trace, warn};

use clap::{Parser, ValueEnum, ArgAction};
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
    Airspy,
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

    /// Verbosity level (-v=info, -vv=debug, -vvv=trace)
    #[arg(short, long, action = ArgAction::Count)]
    verbose: u8,

    /// Output RDS data as JSON (one JSON object per line, redsea-compatible format)
    #[arg(long, default_value_t = false)]
    json: bool,

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

     /// Airspy device index (None for first device)
     #[arg(long)]
     airspy_device_index: Option<usize>,

     /// Airspy LNA gain (0-14, None for default)
     #[arg(long)]
     airspy_lna: Option<u8>,

     /// Airspy mixer gain (0-15, None for default)
     #[arg(long)]
     airspy_mixer: Option<u8>,

     /// Airspy VGA gain (0-15, None for default)
     #[arg(long)]
     airspy_vga: Option<u8>,

    /// Output raw FM-demodulated MPX signal to stdout (for piping to redsea)
    /// Format: signed 16-bit PCM at native MPX rate (use --resample-out for redsea)
    #[arg(long, default_value_t = false)]
    raw_out: bool,

    /// Resample raw output to this rate. Recommended: 171000 (3×57kHz) or 228000 (4×57kHz)
    /// for optimal RDS decoding with redsea
    #[arg(long)]
    resample_out: Option<u32>,

    /// Output raw I/Q after decimation (before FM demod) to stderr for debugging
    #[arg(long, default_value_t = false)]
    dump_iq: bool,
}

const FM_BANDWIDTH: f32 = 240_000.0;
const MONO_SIGNAL_BW: f32 = 15_000.0;
const AUDIO_RATE: usize = 48_000;

#[tokio::main]
async fn main() -> desperado::Result<()> {
    let args = Args::parse();

    // Initialize tracing with verbosity level
    // 0 = WARN (quiet), 1 = INFO, 2 = DEBUG, 3+ = TRACE
    let log_level = match args.verbose {
        0 => tracing::Level::WARN,
        1 => tracing::Level::INFO,
        2 => tracing::Level::DEBUG,
        _ => tracing::Level::TRACE,
    };
    
    let _ = tracing_subscriber::fmt()
        .with_max_level(log_level)
        .with_writer(std::io::stderr)
        .try_init();

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
        SourceType::Airspy => {
            #[cfg(feature = "airspy")]
            {
                let gain = match args.gain {
                    Some(g) => desperado::Gain::Manual(g as f64),
                    None => desperado::Gain::Auto,
                };
                IqAsyncSource::from_airspy(
                    args.airspy_device_index,
                    tuning_freq,
                    args.sample_rate,
                    gain,
                    args.airspy_lna,
                    args.airspy_mixer,
                    args.airspy_vga,
                )
                .await?
            }
            #[cfg(not(feature = "airspy"))]
            {
                eprintln!("Error: airspy feature not enabled. Rebuild with --features airspy");
                std::process::exit(1);
            }
        }
        SourceType::Soapy => {
            #[cfg(feature = "soapy")]
            {
                let gain = match args.gain {
                    Some(g) => desperado::Gain::Manual(g as f64),
                    None => desperado::Gain::Auto,
                };
                IqAsyncSource::from_soapy(
                    &args.soapy_args,
                    args.soapy_channel,
                    tuning_freq,
                    args.sample_rate,
                    gain,
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

    info!(
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
        info!("Audio output disabled (--no-audio)");
        None
    };

    let mut rotate = Rotate::new(-2.0 * PI * args.offset_freq as f32 / args.sample_rate as f32);
    let mut phase_extractor = PhaseExtractor::new();
    let factor = (args.sample_rate as f32 / FM_BANDWIDTH).round() as usize;
    let mut decimator = Decimator::new(factor);
    
    // IMPORTANT: Calculate actual decimated sample rate 
    // factor = round(2000000 / 240000) = 8, so actual rate = 2000000 / 8 = 250000
    let actual_mpx_rate = args.sample_rate as f32 / factor as f32;
    info!("[DSP] Decimation factor: {}, actual MPX rate: {} Hz (target was {} Hz)", 
        factor, actual_mpx_rate, FM_BANDWIDTH);

    let n = 2048;
    let window = 64;
    let mut afc = SquareFreqOffsetCorrection::with_params(n, window, false);

    let lowpass_fir = LowPassFir::new(MONO_SIGNAL_BW, actual_mpx_rate, 256);

    if args.stereo {
        info!("Running stereo FM demodulator with RDS...");
        run_stereo(
            &mut iq_source,
            &args,
            &mut rotate,
            &mut phase_extractor,
            &mut decimator,
            &mut afc,
            &lowpass_fir,
            actual_mpx_rate,
            tx,
        )
        .await?;
    } else {
        info!("Running mono FM demodulator...");
        run_mono(
            &mut iq_source,
            &args,
            &mut rotate,
            &mut phase_extractor,
            &mut decimator,
            &mut afc,
            &lowpass_fir,
            actual_mpx_rate,
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
    mpx_sample_rate: f32,
    tx: channel::Sender<f32>,
) -> desperado::Result<()> {
    let mut deemphasis = DeemphasisFilter::new(mpx_sample_rate, 50e-6);
    let mut audio_resample =
        AudioAdaptiveResampler::new(AUDIO_RATE as f64 / mpx_sample_rate as f64, 1, 1);

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
    _lowpass_fir: &LowPassFir,
    mpx_sample_rate: f32,
    tx: channel::Sender<f32>,
) -> desperado::Result<()> {
    let mut stereo = StereoDecoderPLL::new(mpx_sample_rate);
    let mut deemphasis_l = DeemphasisFilter::new(mpx_sample_rate, 50e-6);
    let mut deemphasis_r = DeemphasisFilter::new(mpx_sample_rate, 50e-6);
    
    // RDS resampler: actual MPX rate → 171 kHz (matches redsea's preferred rate)
    // Note: 171 kHz gives exactly 3 samples per RDS symbol (171000 / 57000 = 3)
    let rds_target_rate = 171_000.0_f32;
    let mut rds_resampler = RdsResamplerCustom::new(mpx_sample_rate, rds_target_rate);
    let mut rds = RdsDecoder::new(rds_target_rate, args.verbose >= 2, args.json);

    let mut audio_resample =
        AudioAdaptiveResampler::new(AUDIO_RATE as f64 / mpx_sample_rate as f64, 5, 2);

    // Real-time pacing for file playback - track absolute time to prevent drift
    let is_file_source = matches!(args.source, SourceType::File);
    
    // Raw output mode: write MPX samples to stdout for piping to redsea
    let raw_out = args.raw_out;
    let mut stdout = std::io::stdout().lock();
    
    // Optional resampler for raw output (actual_mpx_rate -> target rate)
    let mpx_rate_u32 = mpx_sample_rate as u32;
    let mut raw_resampler: Option<rubato::SincFixedOut<f32>> = if raw_out {
        if let Some(target_rate) = args.resample_out {
            if target_rate != mpx_rate_u32 {
                let ratio = target_rate as f64 / mpx_sample_rate as f64;
                let params = SincInterpolationParameters {
                    sinc_len: 128,
                    f_cutoff: 0.9,
                    interpolation: SincInterpolationType::Cubic,
                    oversampling_factor: 128,
                    window: WindowFunction::BlackmanHarris2,
                };
                info!("[RAW-OUT] Resampling from {} Hz to {} Hz", mpx_rate_u32, target_rate);
                Some(SincFixedOut::<f32>::new(ratio, 1.1, params, 1024, 1).unwrap())
            } else {
                None
            }
        } else {
            info!("[RAW-OUT] Outputting at {} Hz (use --resample-out to change)", mpx_rate_u32);
            None
        }
    } else {
        None
    };
    let mut raw_leftover: Vec<f32> = Vec::new();

    while let Some(chunk) = iq_source.next().await {
        let chunk = chunk.map_err(|e| std::io::Error::other(format!("{}", e)))?;

        let shifted = rotate.process(&chunk);
        let decimated = decimator.process(&shifted);
        
        // Debug: dump raw I/Q after decimation to file
        if args.dump_iq {
            use std::io::Write;
            static IQ_FILE: std::sync::OnceLock<std::sync::Mutex<std::fs::File>> = std::sync::OnceLock::new();
            let file = IQ_FILE.get_or_init(|| {
                debug!("[DEBUG] Dumping I/Q to /tmp/fmradio_iq.raw");
                std::sync::Mutex::new(std::fs::File::create("/tmp/fmradio_iq.raw").unwrap())
            });
            let mut f = file.lock().unwrap();
            for sample in &decimated {
                // Write as interleaved I/Q, signed 16-bit
                let i = (sample.re * 32767.0).clamp(-32768.0, 32767.0) as i16;
                let q = (sample.im * 32767.0).clamp(-32768.0, 32767.0) as i16;
                let _ = f.write_all(&i.to_le_bytes());
                let _ = f.write_all(&q.to_le_bytes());
            }
        }
        
        let afc_corrected = if args.afc {
            afc.process(&decimated)
        } else {
            decimated
        };
        let phase = phase_extractor.process(&afc_corrected);

        // Raw output mode: output MPX samples to stdout for piping to redsea
        if raw_out {
            use std::io::Write;
            
            // Optionally resample
            let samples_to_output = if let Some(ref mut resampler) = raw_resampler {
                // Add to leftover buffer
                raw_leftover.extend_from_slice(&phase);
                
                let mut output = Vec::new();
                loop {
                    let needed = resampler.input_frames_next();
                    if raw_leftover.len() < needed {
                        break;
                    }
                    let input_chunk: Vec<Vec<f32>> = vec![raw_leftover[..needed].to_vec()];
                    raw_leftover.drain(0..needed);
                    
                    if let Ok(resampled) = resampler.process(&input_chunk, None)
                        && !resampled.is_empty() {
                            output.extend_from_slice(&resampled[0]);
                        }
                }
                output
            } else {
                phase.clone()
            };
            
            // Convert f32 to signed 16-bit PCM and write to stdout
            // rtl_fm outputs samples scaled similarly to audio - we need to find the right scale
            // Debug: print min/max of first batch
            static DBG_RAW: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
            let dbg_cnt = DBG_RAW.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            if dbg_cnt == 0 && !samples_to_output.is_empty() {
                let min = samples_to_output.iter().cloned().fold(f32::INFINITY, f32::min);
                let max = samples_to_output.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
                let mean = samples_to_output.iter().sum::<f32>() / samples_to_output.len() as f32;
                debug!("[RAW-DBG] Phase signal: min={:.4}, max={:.4}, mean={:.4}, len={}", 
                    min, max, mean, samples_to_output.len());
            }
            
            // Scale: Phase extractor outputs ~±π for FM deviation. 
            // rtl_fm seems to scale to small i16 values (most samples near 0)
            // Scale so that full deviation maps to ~25000 (leaving headroom)
            for sample in samples_to_output {
                let scaled = (sample * 5000.0).clamp(-32768.0, 32767.0) as i16;
                let bytes = scaled.to_le_bytes();
                let _ = stdout.write_all(&bytes);
            }
            let _ = stdout.flush();
            
            // In raw output mode, skip normal processing
            continue;
        }

        // Stereo decoder needs raw phase signal (contains 19 kHz pilot and 38 kHz L-R subcarrier)
        // The stereo decoder has its own pilot bandpass filter (18.3-19.7 kHz)
        let (left, right, pilot_phases) = stereo.process(&phase);
        let deem_l = deemphasis_l.process(&left);
        let deem_r = deemphasis_r.process(&right);

        // RDS: Resample MPX to 171 kHz using pilot-coherent carrier mixing
        // This uses the 19 kHz pilot phase × 3 for perfect 57 kHz carrier lock
        let (rds_i, rds_q) = rds_resampler.process_with_pilot(&phase, &pilot_phases);
        if !rds_i.is_empty() {
            rds.process_iq(&rds_i, &rds_q);
        }

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
                    warn!("Resampler error: {:?}", e);
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
    error_lpf_state: f64,  // IIR lowpass state for phase error filtering
    error_lpf_alpha: f64,  // IIR lowpass coefficient
    pilot_hi: LowPassFir,
    pilot_lo: LowPassFir,
    diff_lowpass: LowPassFir,
}

impl StereoDecoderPLL {
    pub fn new(sample_rate: f32) -> Self {
        let nominal = 19_000.0_f64;
        // IIR lowpass for phase error: fc ~ 50 Hz, alpha = 2*pi*fc / (fs + 2*pi*fc)
        // This filters out the 2*f_pilot component from the multiplier phase detector
        let fc = 50.0;  // Phase error filter cutoff (Hz)
        let error_lpf_alpha = 2.0 * std::f64::consts::PI * fc / (sample_rate as f64 + 2.0 * std::f64::consts::PI * fc);
        
        Self {
            sample_rate,
            pll_phase: 0.0,
            pll_freq: nominal,
            nominal_freq: nominal,
            // PLL gains: kp for proportional (phase), ki for integral (frequency)
            // These are tuned for tracking a stable 19 kHz pilot
            kp: 0.01,    // Proportional gain - controls phase response speed
            ki: 5e-6,    // Integral gain - controls frequency acquisition
            error_lpf_state: 0.0,
            error_lpf_alpha,
            pilot_hi: LowPassFir::new(19_700.0, sample_rate, 257),
            pilot_lo: LowPassFir::new(18_300.0, sample_rate, 257),
            diff_lowpass: LowPassFir::new(15_000.0, sample_rate, 257),
        }
    }

    /// Process input samples and return (left, right, pilot_phases)
    /// pilot_phases contains the 19 kHz pilot PLL phase at each sample (for RDS carrier recovery)
    pub fn process(&mut self, input: &[f32]) -> (Vec<f32>, Vec<f32>, Vec<f64>) {
        let n = input.len();
        let pilot_hi = self.pilot_hi.process(input);
        let pilot_lo = self.pilot_lo.process(input);
        let mut pilot_band = Vec::with_capacity(n);
        for i in 0..n {
            pilot_band.push(pilot_hi[i] - pilot_lo[i]);
        }

        let mut lmr_raw = Vec::with_capacity(n);
        let mut pilot_phases = Vec::with_capacity(n);
        let phase_inc = 2.0 * std::f64::consts::PI / self.sample_rate as f64;
        
        for &p in &pilot_band {
            let p = p as f64;
            
            // Multiplier phase detector for real sinusoid:
            // If pilot = A*cos(ω*t + φ) and NCO = sin(pll_phase), then
            // product = A*cos(ω*t + φ) * sin(pll_phase)
            //         = A/2 * [sin(pll_phase - ω*t - φ) + sin(pll_phase + ω*t + φ)]
            // After lowpass filtering, we get: A/2 * sin(phase_error)
            // where phase_error = pll_phase - (ω*t + φ)
            // When locked: phase_error → 0, so error → 0
            let raw_error = p * self.pll_phase.sin();
            
            // IIR lowpass filter to remove 2*f_pilot component
            self.error_lpf_state += self.error_lpf_alpha * (raw_error - self.error_lpf_state);
            let error = self.error_lpf_state;

            // Second-order PLL update: frequency integrator + phase
            self.pll_freq += self.ki * error;
            self.pll_freq = self
                .pll_freq
                .clamp(self.nominal_freq * 0.99, self.nominal_freq * 1.01);
            
            // Phase update: NCO advance + proportional correction
            self.pll_phase += phase_inc * self.pll_freq + self.kp * error;
            
            // Keep phase in [-π, π]
            while self.pll_phase > std::f64::consts::PI {
                self.pll_phase -= 2.0 * std::f64::consts::PI;
            }
            while self.pll_phase < -std::f64::consts::PI {
                self.pll_phase += 2.0 * std::f64::consts::PI;
            }

            // Store the pilot phase at this sample AFTER the PLL update
            pilot_phases.push(self.pll_phase);

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
        (left, right, pilot_phases)
    }
    
    /// Get the current PLL phase (for 19 kHz pilot)
    /// Multiply by 3 to get phase for 57 kHz RDS carrier
    pub fn pilot_phase(&self) -> f64 {
        self.pll_phase
    }
    
    /// Get the current PLL frequency (should be ~19 kHz when locked)
    pub fn pilot_freq(&self) -> f64 {
        self.pll_freq
    }
}

/// Stateful FIR filter with ring buffer for sample-by-sample processing
struct StatefulFir {
    coeffs: Vec<f32>,
    buffer: Vec<f32>,
    write_pos: usize,
}

impl StatefulFir {
    fn new(cutoff_freq: f32, sample_rate: f32, taps: usize) -> Self {
        // Design windowed-sinc filter (same as LowPassFir)
        let mut coeffs = Vec::with_capacity(taps);
        let mid = (taps / 2) as isize;
        let norm_cutoff = cutoff_freq / (sample_rate / 2.0);
        
        for n in 0..taps {
            let x = n as isize - mid;
            let sinc = if x == 0 {
                2.0 * norm_cutoff
            } else {
                (2.0 * norm_cutoff * PI * x as f32).sin() / (PI * x as f32)
            };
            // Blackman window
            let window = 0.42 - 0.5 * ((2.0 * PI * n as f32) / (taps as f32 - 1.0)).cos()
                + 0.08 * ((4.0 * PI * n as f32) / (taps as f32 - 1.0)).cos();
            coeffs.push(sinc * window);
        }
        
        // Normalize
        let norm: f32 = coeffs.iter().sum();
        for v in coeffs.iter_mut() {
            *v /= norm;
        }
        
        Self {
            coeffs,
            buffer: vec![0.0; taps],
            write_pos: 0,
        }
    }
    
    fn push(&mut self, sample: f32) -> f32 {
        // Add sample to ring buffer
        self.buffer[self.write_pos] = sample;
        self.write_pos = (self.write_pos + 1) % self.buffer.len();
        
        // Compute filtered output (convolution)
        let mut acc = 0.0_f32;
        let len = self.coeffs.len();
        for i in 0..len {
            let buf_idx = (self.write_pos + len - 1 - i) % len;
            acc += self.buffer[buf_idx] * self.coeffs[i];
        }
        acc
    }
}

// Note: RrcFilter and PolyphaseSymSync structs have been removed.
// Now using desperado::dsp::symsync::SymSync instead.

/// RDS resampler: input rate → output rate
/// Default: 250 kHz → 171 kHz (exactly 3 samples per RDS symbol after decimation by 24)
/// This matches redsea's architecture: 171000 / 24 = 7125 Hz, 7125 / 2375 = 3.0 exactly
/// Now also does coherent mixing down using pilot-derived 57 kHz carrier
struct RdsResamplerCustom {
    resampler_i: SincFixedOut<f32>,
    resampler_q: SincFixedOut<f32>,
    leftover_i: Vec<f32>,
    leftover_q: Vec<f32>,
    input_rate: f32,
}

impl RdsResamplerCustom {
    fn new(input_rate: f32, output_rate: f32) -> Self {
        let ratio = output_rate as f64 / input_rate as f64;
        
        let params_i = SincInterpolationParameters {
            sinc_len: 128,          // Good quality for this ratio
            f_cutoff: 0.9,          // Cutoff just below Nyquist
            interpolation: SincInterpolationType::Cubic,
            oversampling_factor: 128,
            window: WindowFunction::BlackmanHarris2,
        };
        
        let params_q = SincInterpolationParameters {
            sinc_len: 128,
            f_cutoff: 0.9,
            interpolation: SincInterpolationType::Cubic,
            oversampling_factor: 128,
            window: WindowFunction::BlackmanHarris2,
        };
        
        // Output 1024 frames at a time (1 channel each for I and Q)
        let resampler_i = SincFixedOut::<f32>::new(ratio, 1.1, params_i, 1024, 1).unwrap();
        let resampler_q = SincFixedOut::<f32>::new(ratio, 1.1, params_q, 1024, 1).unwrap();
        
        debug!("[RDS-RESAMP] Created {}kHz → {}kHz complex resampler (ratio: {:.4})", 
            input_rate / 1000.0, output_rate / 1000.0, ratio);
        
        Self {
            resampler_i,
            resampler_q,
            leftover_i: Vec::new(),
            leftover_q: Vec::new(),
            input_rate,
        }
    }
    
    /// Process MPX signal with pilot phases to produce coherent baseband I/Q
    /// pilot_phases: phase of 19 kHz pilot at each sample (multiply by 3 for 57 kHz)
    fn process_with_pilot(&mut self, input: &[f32], pilot_phases: &[f64]) -> (Vec<f32>, Vec<f32>) {
        // Mix down to baseband using pilot-derived 57 kHz carrier
        // 57 kHz = 3 × 19 kHz, so phase_57 = 3 × phase_19
        // The RDS carrier may have a phase offset relative to the pilot
        // Try adding 90° (π/2) to compensate
        let mut i_mixed = Vec::with_capacity(input.len());
        let mut q_mixed = Vec::with_capacity(input.len());
        
        // Debug: print pilot phase statistics once
        static DEBUG_COUNTER: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
        let count = DEBUG_COUNTER.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        if count == 100 && pilot_phases.len() >= 10 {
            // Check pilot phase rate of change using average over several samples to avoid wrap issues
            let mut total_delta = 0.0;
            let mut valid_deltas = 0;
            for i in 1..pilot_phases.len().min(100) {
                let mut delta = pilot_phases[i] - pilot_phases[i-1];
                // Handle phase wrapping
                if delta > std::f64::consts::PI {
                    delta -= 2.0 * std::f64::consts::PI;
                } else if delta < -std::f64::consts::PI {
                    delta += 2.0 * std::f64::consts::PI;
                }
                total_delta += delta;
                valid_deltas += 1;
            }
            let avg_delta = if valid_deltas > 0 { total_delta / valid_deltas as f64 } else { 0.0 };
            let measured_freq = avg_delta * self.input_rate as f64 / (2.0 * std::f64::consts::PI);
            let expected_delta = 2.0 * std::f64::consts::PI * 19000.0 / self.input_rate as f64;
            trace!("[RDS-PILOT-DBG] avg_delta: {:.5}, expected: {:.5}, measured_freq: {:.1} Hz",
                avg_delta, expected_delta, measured_freq);
        }
        
        for (idx, &x) in input.iter().enumerate() {
            let pilot_phase = pilot_phases.get(idx).copied().unwrap_or(0.0);
            // The stereo decoder PLL locks sin(pll_phase) to the pilot
            // For 57 kHz, we need to adjust phase relationship
            // Try using sin() for I instead of cos() to match the stereo carrier convention
            let phase_57 = 3.0 * pilot_phase;  // 57 kHz = 3 × 19 kHz pilot
            
            // Mix down: use sin for I channel (matching 38 kHz convention)
            let sin_val = phase_57.sin() as f32;
            let cos_val = phase_57.cos() as f32;
            
            i_mixed.push(x * sin_val);
            q_mixed.push(x * cos_val);
        }
        
        // Add to leftovers
        self.leftover_i.extend_from_slice(&i_mixed);
        self.leftover_q.extend_from_slice(&q_mixed);
        
        let mut output_i = Vec::new();
        let mut output_q = Vec::new();
        
        loop {
            let input_frames_needed = self.resampler_i.input_frames_next();
            
            if self.leftover_i.len() < input_frames_needed {
                break;
            }
            
            // Take exactly what we need for I
            let input_chunk_i: Vec<Vec<f32>> = vec![self.leftover_i[..input_frames_needed].to_vec()];
            let input_chunk_q: Vec<Vec<f32>> = vec![self.leftover_q[..input_frames_needed].to_vec()];
            self.leftover_i.drain(0..input_frames_needed);
            self.leftover_q.drain(0..input_frames_needed);
            
            // Process I
            match self.resampler_i.process(&input_chunk_i, None) {
                Ok(resampled) => {
                    if !resampled.is_empty() {
                        output_i.extend_from_slice(&resampled[0]);
                    }
                }
                Err(e) => {
                    warn!("[RDS-RESAMP] Error I: {:?}", e);
                    break;
                }
            }
            
            // Process Q
            match self.resampler_q.process(&input_chunk_q, None) {
                Ok(resampled) => {
                    if !resampled.is_empty() {
                        output_q.extend_from_slice(&resampled[0]);
                    }
                }
                Err(e) => {
                    warn!("[RDS-RESAMP] Error Q: {:?}", e);
                    break;
                }
            }
        }
        
        (output_i, output_q)
    }
    
    /// Simple real-only resampling (for use with independent NCO approach)
    #[allow(dead_code)]
    fn process_real(&mut self, input: &[f32]) -> Vec<f32> {
        // Use I resampler for real signal
        self.leftover_i.extend_from_slice(input);
        
        let mut output = Vec::new();
        
        loop {
            let input_frames_needed = self.resampler_i.input_frames_next();
            
            if self.leftover_i.len() < input_frames_needed {
                break;
            }
            
            let input_chunk: Vec<Vec<f32>> = vec![self.leftover_i[..input_frames_needed].to_vec()];
            self.leftover_i.drain(0..input_frames_needed);
            
            match self.resampler_i.process(&input_chunk, None) {
                Ok(resampled) => {
                    if !resampled.is_empty() {
                        output.extend_from_slice(&resampled[0]);
                    }
                }
                Err(e) => {
                    warn!("[RDS-RESAMP] Error: {:?}", e);
                    break;
                }
            }
        }
        
        output
    }
}

/// RDS decoder with NCO mixer, PLL, and proper biphase decoding (matching redsea architecture)
pub struct RdsDecoder {
    sample_rate: f32,
    
    // NCO with integrated PLL (from desperado::dsp::nco)
    nco: Nco,
    
    // Lowpass filter for baseband (complex I/Q)
    lpf_i: StatefulFir,
    lpf_q: StatefulFir,
    
    // AGC (from desperado::dsp::agc)
    agc: Agc,
    
    // Polyphase symbol synchronizer (from desperado::dsp::symsync)
    symsync: SymSync,
    
    // Decimation
    decimate_ratio: usize,
    decimate_counter: usize,
    
    // Biphase decoder state
    prev_psk_symbol: (f32, f32),
    biphase_clock: usize,
    biphase_polarity: usize,
    biphase_history: Vec<f32>,
    
    // Delta decoder
    prev_biphase_bit: bool,
    
    // Bit output
    bits: Vec<u8>,
    rds_parser: RdsParser,
    
    // Display state
    last_station: Option<String>,
    last_radiotext: Option<String>,
    last_pty: u8,
    last_di_flags: DIFlags,
    last_af_list: Vec<u16>,
    debug_counter: u64,
    
    // PLL acquisition state
    pll_locked: bool,           // True when PLL has acquired lock
    pll_lock_counter: usize,    // Count consecutive low phase errors
    
    // JSON output mode
    json_mode: bool,
}

impl RdsDecoder {
    /// Create RDS decoder expecting 171 kHz input (from RdsResampler)
    /// This matches redsea's architecture exactly:
    /// - 171 kHz input rate
    /// - Decimate by 24 → 7125 Hz
    /// - 7125 / 2375 = 3.0 samples per symbol exactly
    pub fn new(_sample_rate: f32, verbose: bool, json: bool) -> Self {
        let mut rds_parser = RdsParser::new();
        rds_parser.set_verbose(verbose);
        rds_parser.set_json_mode(json);
        
        // Fixed parameters matching redsea exactly
        let sample_rate = 171_000.0_f32;  // We now expect 171 kHz input (from resampler)
        let decimate_ratio = 24_usize;     // Fixed: 171000 / 24 = 7125 Hz
        let decimated_rate = 7125.0_f32;   // Exactly 3 samples per 2375 Hz symbol
        let psk_symbol_rate = 2375.0_f32;
        
        debug!("[RDS-DSP] Sample rate: {} Hz, Decimate ratio: {}, Decimated rate: {} Hz",
            sample_rate, decimate_ratio, decimated_rate);
        debug!("[RDS-DSP] Samples per PSK symbol: {:.2}, PSK rate: {:.1} Hz", 
            decimated_rate / psk_symbol_rate, psk_symbol_rate);
        
        // Lowpass at 2400 Hz (matching redsea)
        let lpf_cutoff = 2400.0;
        // Use 255 taps for good selectivity
        let lpf_taps = 255;
        
        // NCO for fine phase tracking (IQ path: 57 kHz already removed by pilot-coherent mixing)
        // In IQ mode, we only need to track residual phase, so frequency is 0
        // The NCO runs at the DECIMATED rate (7125 Hz) for phase tracking at symbol rate
        let decimated_rate = 7125.0_f64;
        let mut nco = Nco::new(0.0, decimated_rate);  // Zero frequency for IQ path
        // PLL bandwidth for phase tracking (at 7125 Hz rate, 2375 symbol rate)
        // Use ~2 Hz effective bandwidth for stable tracking
        // At symbol rate ~2375 Hz, we update every ~3 samples, so effective = nominal * (2375/7125) / 3 ≈ nominal * 0.11
        nco.set_pll_bandwidth(20.0, decimated_rate);  // ~2.2 Hz effective at symbol rate
        
        debug!("[RDS-DSP] NCO: freq=0 Hz (IQ path), PLL bandwidth=20Hz (~2Hz effective)");
        
        // AGC with bandwidth matching redsea (~500 Hz at 171 kHz = 0.003)
        // redsea: agc.init(kAGCBandwidth_Hz / kTargetSampleRate_Hz, kAGCInitialGain)
        // kAGCBandwidth_Hz = 500.0, so 500/171000 ≈ 0.003
        // kAGCInitialGain = 0.08
        let mut agc = Agc::new(0.003);
        agc.set_gain(0.08);  // Match redsea initial gain
        debug!("[RDS-DSP] AGC: bandwidth=0.003, initial_gain=0.08");
        
        // Polyphase symbol synchronizer (matching redsea's liquid-dsp symsync)
        // redsea: symsync.init(LIQUID_FIRFILT_RRC, kSamplesPerSymbol, kSymsyncDelay, kSymsyncBeta, 32)
        //         symsync.setBandwidth(kSymsyncBandwidth_Hz / kTargetSampleRate_Hz)
        // kSymsyncBandwidth_Hz = 2200.0, so 2200/171000 ≈ 0.013
        let symsync = SymSync::new_rnyquist(3, 32, 3, 0.8, 0.013);
        debug!("[RDS-DSP] SymSync: k=3, npfb=32, m=3, beta=0.8, bandwidth=0.013");
        
        Self {
            sample_rate,
            nco,
            lpf_i: StatefulFir::new(lpf_cutoff, sample_rate, lpf_taps),
            lpf_q: StatefulFir::new(lpf_cutoff, sample_rate, lpf_taps),
            agc,
            symsync,
            decimate_ratio,
            decimate_counter: 0,
            prev_psk_symbol: (0.0, 0.0),
            biphase_clock: 0,
            biphase_polarity: 0,
            biphase_history: vec![0.0; 128],
            prev_biphase_bit: false,
            bits: Vec::new(),
            rds_parser,
            last_station: None,
            last_radiotext: None,
            last_pty: 255,
            last_di_flags: DIFlags::default(),
            last_af_list: Vec::new(),
            debug_counter: 0,
            pll_locked: false,
            pll_lock_counter: 0,
            json_mode: json,
        }
    }
    
    pub fn process(&mut self, input: &[f32]) {
        // Debug: check input signal statistics
        if self.debug_counter == 0 {
            let input_rms: f32 = (input.iter().map(|x| x*x).sum::<f32>() / input.len() as f32).sqrt();
            debug!("[RDS-DSP] Input RMS: {:.4}, Input len: {}", input_rms, input.len());
        }
        
        // Process each sample through the DSP chain with Costas loop PLL
        
        for &x in input {
            // Step 1: Mix down to baseband using NCO
            let (i_mixed, q_mixed) = self.nco.mix_down(x);
            
            // Step 2: Lowpass filter (applied at full rate, before decimation)
            let i_filt = self.lpf_i.push(i_mixed);
            let q_filt = self.lpf_q.push(q_mixed);
            
            // Step 3: Advance NCO phase
            self.nco.step();
            
            // Step 4: Decimation - only process every N samples
            self.decimate_counter += 1;
            if self.decimate_counter < self.decimate_ratio {
                continue;
            }
            self.decimate_counter = 0;
            
            // Step 5: AGC (using new AGC module)
            let (i_agc, q_agc) = self.agc.execute(i_filt, q_filt);
            
            // Count decimated samples for rate verification
            static DECIMATED_COUNT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
            let dec_count = DECIMATED_COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            if dec_count.is_multiple_of(50000) && dec_count > 0 {
                debug!("[RDS-DEC] {} decimated samples processed", dec_count);
            }
            
            // Debug: check signal level after AGC (once every ~1 second)
            if self.debug_counter % 7000 == 1 {
                let pre_mag = (i_filt * i_filt + q_filt * q_filt).sqrt();
                let agc_mag = (i_agc * i_agc + q_agc * q_agc).sqrt();
                trace!("[RDS-SIG] pre-AGC mag: {:.4}, AGC gain: {:.2}, post-AGC mag: {:.4}",
                    pre_mag, self.agc.get_gain(), agc_mag);
            }
            
            // Step 6: Polyphase symbol synchronizer
            let symbol_opt = self.symsync.push(i_agc, q_agc);
            
            // Only process when we have a symbol output from symsync
            let psk_symbol = match symbol_opt {
                Some(sym) => sym,
                None => continue,
            };
            
            // Carrier PLL: Use liquid-dsp BPSK modem phase error calculation
            // liquid-dsp formula: phase_error = imag(r * conj(x_hat))
            // For BPSK: x_hat = +1 or -1, so phase_error = Q * sign(I)
            let (si, sq) = (psk_symbol.0 as f64, psk_symbol.1 as f64);
            let sym_mag = (si * si + sq * sq).sqrt();
            
            // Lower threshold to 0.001 - any signal is useful for phase tracking
            if sym_mag > 0.001 {
                // Compute phase error as angle from nearest BPSK constellation point (0° or 180°)
                // For symbol at angle θ:
                //   If I >= 0: it's a +1 symbol, error = θ - 0° = θ
                //   If I < 0: it's a -1 symbol, error = θ - 180° (wrapped to ±π)
                let symbol_phase = sq.atan2(si);  // Phase in radians
                let phase_error_rad = if si >= 0.0 {
                    // Should be at 0°
                    symbol_phase
                } else {
                    // Should be at 180° (π radians)
                    // Wrap to [-π, π]
                    let err = symbol_phase - std::f64::consts::PI;
                    if err < -std::f64::consts::PI {
                        err + 2.0 * std::f64::consts::PI
                    } else if err > std::f64::consts::PI {
                        err - 2.0 * std::f64::consts::PI
                    } else {
                        err
                    }
                };
                
                // Phase error is already in [-π, π] from the calculation above
                // Clamp just for safety
                
                // Debug: print phase error distribution
                static PE_COUNT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
                let pe_count = PE_COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                if pe_count.is_multiple_of(5000) && pe_count > 0 {
                    let phase_deg = sq.atan2(si).to_degrees();
                    debug!("[RDS-PLL-DBG] mag: {:.3}, phase: {:+.1}°, phase_err: {:+.3} rad ({:+.1}°), I: {:+.3}, Q: {:+.3}",
                        sym_mag, phase_deg, phase_error_rad, phase_error_rad.to_degrees(), si, sq);
                }
                
                // PLL multiplier: Reduced gain since phase error is now accurate
                let scaled_error = phase_error_rad * 0.5;
                
                // Use NCO's integrated PLL (now matches liquid-dsp exactly)
                self.nco.pll_step(scaled_error);
                
                // Lock detection: if phase error is consistently small
                let phase_error_deg = phase_error_rad.to_degrees();
                if phase_error_deg.abs() < 20.0 {
                    self.pll_lock_counter += 1;
                    if self.pll_lock_counter > 500 && !self.pll_locked {
                        self.pll_locked = true;
                        // Narrow bandwidth once locked for better tracking
                        // 2.16 Hz nominal = 0.03 Hz effective at symbol rate
                        self.nco.set_pll_bandwidth(2.16, self.sample_rate as f64);
                        debug!("[RDS-PLL] Locked! Narrowing bandwidth to 2.16 Hz (~0.03 Hz effective). Phase error: {:.1}°, freq: {:.2} Hz", 
                            phase_error_deg, self.nco.get_frequency_hz(self.sample_rate as f64));
                    }
                } else {
                    if self.pll_lock_counter > 0 {
                        self.pll_lock_counter -= 1;
                    }
                    if self.pll_lock_counter == 0 && self.pll_locked {
                        self.pll_locked = false;
                        // Widen bandwidth for re-acquisition
                        self.nco.set_pll_bandwidth(100.0, self.sample_rate as f64);
                        debug!("[RDS-PLL] Lost lock, widening bandwidth to 100 Hz. Phase error: {:.1}°", phase_error_deg);
                    }
                }
            }
            
            self.debug_counter += 1;
            // Count symbols for rate verification
            static SYMBOL_COUNT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
            let count = SYMBOL_COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            if count.is_multiple_of(10000) && count > 0 {
                debug!("[RDS-RATE] {} symbols processed", count);
            }
            if self.debug_counter.is_multiple_of(5000) {
                let mag = (psk_symbol.0 * psk_symbol.0 + psk_symbol.1 * psk_symbol.1).sqrt();
                let phase_deg = (psk_symbol.1 as f64).atan2(psk_symbol.0 as f64).to_degrees();
                debug!("[RDS-DSP] AGC: {:.2}, NCO: {:.2} Hz, I: {:.3}, Q: {:.3}, mag: {:.3}, phase: {:.1}°, locked: {}, symsync: {:.4}",
                    self.agc.get_gain(), self.nco.get_frequency_hz(self.sample_rate as f64), 
                    psk_symbol.0, psk_symbol.1, mag, phase_deg, self.pll_locked, self.symsync.get_rate());
            }
            
            // Debug: Print constellation points for phase analysis (30 consecutive symbols)
            if self.debug_counter >= 30000 && self.debug_counter < 30030 {
                let prev_phase = (self.prev_psk_symbol.1 as f64).atan2(self.prev_psk_symbol.0 as f64);
                let curr_phase = (psk_symbol.1 as f64).atan2(psk_symbol.0 as f64);
                let phase_diff = (curr_phase - prev_phase).to_degrees();
                // Wrap to -180..180
                let phase_diff = if phase_diff > 180.0 { phase_diff - 360.0 } else if phase_diff < -180.0 { phase_diff + 360.0 } else { phase_diff };
                trace!("[RDS-CONST] I: {:+.3}, Q: {:+.3}, mag: {:.3}, phase: {:+.1}°, Δphase: {:+.1}°",
                    psk_symbol.0, psk_symbol.1, 
                    (psk_symbol.0*psk_symbol.0 + psk_symbol.1*psk_symbol.1).sqrt(),
                    (psk_symbol.1 as f64).atan2(psk_symbol.0 as f64).to_degrees(),
                    phase_diff);
            }
            
            // Biphase decoding using subtraction (matching redsea)
            // biphase_symbol = (current - prev) * 0.5
            // For correct biphase decoding, we need the PLL to lock absolute phase.
            let (pi, pq) = self.prev_psk_symbol;
            let (ci, cq) = psk_symbol;
            let biphase_i = (ci - pi) * 0.5;
            let biphase_q = (cq - pq) * 0.5;
            
            // Bit value is determined by sign of real part (works when PLL locks phase)
            // When phase not locked, we also compute differential product as fallback
            let diff_product = ci * pi + cq * pq;  // cos(Δphase) * mag²
            
            // Use differential product sign for bit decision (more robust to phase rotation)
            // diff_product > 0 means same phase (no flip), < 0 means opposite phase (flip)
            let biphase_bit = diff_product >= 0.0;
            
            // For clock polarity detection, use the biphase subtraction magnitude
            // This works because even when phase rotates, the magnitude of transitions
            // should still show the biphase pattern
            let biphase_mag = (biphase_i * biphase_i + biphase_q * biphase_q).sqrt();
            let history_idx = self.biphase_clock % 128;
            self.biphase_history[history_idx] = biphase_mag;
            
            self.biphase_clock += 1;
            
            // Every 128 symbols, check which clock phase has better signal
            if self.biphase_clock.is_multiple_of(128) && self.biphase_clock > 0 {
                let mut even_sum = 0.0_f32;
                let mut odd_sum = 0.0_f32;
                for i in 0..64 {
                    even_sum += self.biphase_history[i * 2];
                    odd_sum += self.biphase_history[i * 2 + 1];
                }
                let old_polarity = self.biphase_polarity;
                if even_sum > odd_sum {
                    self.biphase_polarity = 0;
                } else if odd_sum > even_sum {
                    self.biphase_polarity = 1;
                }
                // Debug: print polarity choice occasionally
                if self.debug_counter % 30000 < 200 && old_polarity != self.biphase_polarity {
                    trace!("[RDS-BIPH-CLK] even: {:.1}, odd: {:.1}, polarity: {} -> {}",
                        even_sum, odd_sum, old_polarity, self.biphase_polarity);
                }
            }
            
            // Output bit on correct clock phase (delta decode: XOR with previous)
            if self.biphase_clock % 2 == self.biphase_polarity {
                // Use biphase subtraction bit (original redsea approach)
                let decoded_bit = biphase_bit != self.prev_biphase_bit;
                self.prev_biphase_bit = biphase_bit;
                self.bits.push(if decoded_bit { 1 } else { 0 });
            }
            
            // Debug: print differential product occasionally
            if self.debug_counter % 10000 == 1 {
                let prev_phase = (pq as f64).atan2(pi as f64).to_degrees();
                let curr_phase = (cq as f64).atan2(ci as f64).to_degrees();
                let mut phase_diff = curr_phase - prev_phase;
                if phase_diff > 180.0 { phase_diff -= 360.0; }
                if phase_diff < -180.0 { phase_diff += 360.0; }
                trace!("[RDS-BIPH] diff_product: {:+.3}, biph_bit: {}, Δphase: {:+.1}°",
                    diff_product, biphase_bit, phase_diff);
            }
            
            self.prev_psk_symbol = psk_symbol;
        }

        // Feed to RDS parser
        self.process_bits();
    }
    
    /// Process pre-mixed complex I/Q baseband (already at 171 kHz, coherent with pilot)
    /// This skips the NCO mixing and uses the pilot-derived 57 kHz carrier
    pub fn process_iq(&mut self, input_i: &[f32], input_q: &[f32]) {
        if input_i.is_empty() {
            return;
        }
        
        // Debug: check input signal statistics
        static IQ_DBG_COUNTER: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
        let dbg_count = IQ_DBG_COUNTER.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        if dbg_count.is_multiple_of(50) {
            let input_rms: f32 = (input_i.iter().map(|x| x*x).sum::<f32>() / input_i.len() as f32).sqrt();
            debug!("[RDS-DSP-IQ] Input RMS I: {:.6}, Q: {:.6}, len: {}", 
                input_rms,
                (input_q.iter().map(|x| x*x).sum::<f32>() / input_q.len() as f32).sqrt(),
                input_i.len());
        }
        
        // Process each sample through the DSP chain
        // Mixing already done by caller using pilot-derived carrier
        for idx in 0..input_i.len() {
            let i_in = input_i[idx];
            let q_in = input_q[idx];
            
            // Step 1: Lowpass filter (at 171 kHz rate)
            let i_filt = self.lpf_i.push(i_in);
            let q_filt = self.lpf_q.push(q_in);
            
            // Step 2: Decimation - only process every N samples
            self.decimate_counter += 1;
            if self.decimate_counter < self.decimate_ratio {
                continue;
            }
            self.decimate_counter = 0;
            
            // Step 3: AGC (using new AGC module)
            let (i_agc, q_agc) = self.agc.execute(i_filt, q_filt);
            
            // Debug: check signal levels at decimated rate
            static DEC_DBG_COUNTER: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
            let dec_count = DEC_DBG_COUNTER.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            if dec_count.is_multiple_of(5000) {
                let filt_mag = (i_filt * i_filt + q_filt * q_filt).sqrt();
                let agc_mag = (i_agc * i_agc + q_agc * q_agc).sqrt();
                debug!("[RDS-DSP-IQ-LEVELS] Filt mag: {:.6}, AGC gain: {:.2}, AGC out mag: {:.6}",
                    filt_mag, self.agc.get_gain(), agc_mag);
            }
            
            // Step 4: Fine carrier phase tracking
            // The pilot-derived 57 kHz gives us frequency lock, but we need to track the phase
            // Use a simple PLL to rotate symbols to the real axis
            // Complex rotation: (I + jQ) * e^(-j*theta) = (I + jQ) * (cos - j*sin)
            //   = I*cos + Q*sin + j*(Q*cos - I*sin)
            let (sin_rot, cos_rot) = self.nco.sincos();
            let i_rot = i_agc * cos_rot + q_agc * sin_rot;
            let q_rot = q_agc * cos_rot - i_agc * sin_rot;
            
            // Step carrier NCO (at decimated rate, 7125 Hz)
            self.nco.step();
            
            // Step 5: Polyphase symbol synchronizer (replaces old timing buffer + M&M TED)
            // Debug: check signal going into symsync
            static SYMSYNC_DBG_COUNTER: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
            let ss_count = SYMSYNC_DBG_COUNTER.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            if ss_count.is_multiple_of(5000) {
                let rot_mag = (i_rot * i_rot + q_rot * q_rot).sqrt();
                debug!("[RDS-PRE-SYMSYNC] Input to symsync: I={:.4}, Q={:.4}, mag={:.4}", i_rot, q_rot, rot_mag);
            }
            let symbol_opt = self.symsync.push(i_rot, q_rot);
            
            // Only process when we have a symbol
            if let Some(psk_symbol) = symbol_opt {
                let psk_symbol = (psk_symbol.0, psk_symbol.1);
                
                // Update carrier PLL based on symbol phase error
                // For BPSK, we want symbols at ±1 on the real axis
                // Phase error = Q * sign(I) gives us direction to rotate
                // Normalize by magnitude for more stable tracking
                let mag_sq = psk_symbol.0 * psk_symbol.0 + psk_symbol.1 * psk_symbol.1;
                if mag_sq > 0.01 {
                    // Phase error: Q component indicates how far off from real axis
                    // For BPSK: phase_error = atan2(Q, |I|) ≈ Q/|I| for small angles
                    let phase_error = psk_symbol.1 * psk_symbol.0.signum();
                    // Normalize by magnitude for stable operation
                    let phase_error_normalized = (phase_error / mag_sq.sqrt()) as f64;
                    // PLL update at symbol rate
                    self.nco.pll_step(phase_error_normalized);
                }
                
                // Debug output
                self.debug_counter += 1;
                if self.debug_counter.is_multiple_of(5000) {
                    let mag = (psk_symbol.0 * psk_symbol.0 + psk_symbol.1 * psk_symbol.1).sqrt();
                    let sym_phase = psk_symbol.1.atan2(psk_symbol.0).to_degrees();
                    debug!("[RDS-DSP-IQ] AGC: {:.2}, I: {:.3}, Q: {:.3}, mag: {:.3}, sym_ph: {:.1}°",
                        self.agc.get_gain(), psk_symbol.0, psk_symbol.1, mag, sym_phase);
                }
                
                // Print constellation occasionally for phase analysis
                if self.debug_counter % 10000 == 1 {
                    debug!("[RDS-CONST-IQ] Symbol I: {:.3}, Q: {:.3}", psk_symbol.0, psk_symbol.1);
                }
                
                // Biphase decoding using subtraction (matching redsea exactly)
                // With PLL tracking, symbols are at ±1 on the real axis
                let (pi, pq) = self.prev_psk_symbol;
                let (ci, cq) = psk_symbol;
                let biphase_i = (ci - pi) * 0.5;
                let biphase_q = (cq - pq) * 0.5;
                
                // Bit value is determined by sign of real part
                let biphase_bit = biphase_i >= 0.0;
                
                // Store magnitude in history for clock polarity detection
                let history_idx = self.biphase_clock % 128;
                self.biphase_history[history_idx] = biphase_i.abs();
                
                self.biphase_clock += 1;
                
                // Every 128 symbols, check which clock phase has better signal
                if self.biphase_clock >= 128 {
                    let mut even_sum = 0.0_f32;
                    let mut odd_sum = 0.0_f32;
                    for i in 0..64 {
                        even_sum += self.biphase_history[i * 2];
                        odd_sum += self.biphase_history[i * 2 + 1];
                    }
                    debug!("[RDS-CLOCK-IQ] Even: {:.3}, Odd: {:.3}, Ratio: {:.2}, Current polarity: {}",
                        even_sum, odd_sum, even_sum.max(odd_sum) / (even_sum.min(odd_sum) + 0.001), self.biphase_polarity);
                    // Note: Due to clock increment happening before output check,
                    // if even history indices have larger values, we need polarity=1
                    // to output at odd clock-after-increment values (which read from even history)
                    if even_sum > odd_sum {
                        self.biphase_polarity = 1;  // Output at odd clock (reads from even history)
                    } else if odd_sum > even_sum {
                        self.biphase_polarity = 0;  // Output at even clock (reads from odd history)
                    }
                    // Reset clock and history (matching redsea)
                    self.biphase_clock = 0;
                    for i in 0..128 {
                        self.biphase_history[i] = 0.0;
                    }
                }
                
                // Output bit on correct clock phase (delta decode: XOR with previous)
                if self.biphase_clock % 2 == self.biphase_polarity {
                    let decoded_bit = biphase_bit != self.prev_biphase_bit;
                    self.prev_biphase_bit = biphase_bit;
                    // Delta decoded bit (standard RDS differential decoding)
                    self.bits.push(if decoded_bit { 1 } else { 0 });
                }
                
                // Debug: print biphase symbol magnitude occasionally
                // Also print detailed sequence for first 200 symbols
                static BIPHASE_DBG: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
                let biphase_dbg = BIPHASE_DBG.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                if (1000..1100).contains(&biphase_dbg) {
                    debug!("[RDS-BIPH-SEQ] sym={}, clock={}, pol={}, biphase_i={:.3}, bit={}, output={}",
                        biphase_dbg, self.biphase_clock, self.biphase_polarity, biphase_i, biphase_bit,
                        if self.biphase_clock % 2 == self.biphase_polarity { "Y" } else { "N" });
                }
                if self.debug_counter % 10000 == 1 {
                    let biphase_mag = (biphase_i * biphase_i + biphase_q * biphase_q).sqrt();
                    trace!("[RDS-BIPH-IQ] biphase_i: {:.3}, biphase_q: {:.3}, mag: {:.3}, bit: {}",
                        biphase_i, biphase_q, biphase_mag, biphase_bit);
                }
                
                self.prev_psk_symbol = psk_symbol;
            }
        }

        // Feed to RDS parser
        self.process_bits();
    }
    
    /// Process accumulated bits through the RDS parser and display results
    fn process_bits(&mut self) {
        if self.bits.len() >= 104 {
            // Debug: print bit statistics and pattern periodically
            static BIT_BATCH_COUNT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
            let batch_count = BIT_BATCH_COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            if batch_count.is_multiple_of(100) && batch_count > 0 {
                let ones: usize = self.bits.iter().map(|&b| b as usize).sum();
                let zeros = self.bits.len() - ones;
                // Show first 52 bits (2 blocks worth) as string
                let first_bits: String = self.bits.iter().take(52).map(|&b| if b == 1 { '1' } else { '0' }).collect();
                debug!("[RDS-BITS] Batch {}: {} bits, {} ones, {} zeros, ratio: {:.2}, first 52: {}",
                    batch_count, self.bits.len(), ones, zeros, ones as f32 / self.bits.len() as f32, first_bits);
            }
            
            self.rds_parser.push_bits(&self.bits);

            // In JSON mode, output JSON objects and skip human-readable output
            if self.json_mode {
                for json_out in self.rds_parser.take_json_outputs() {
                    if let Ok(json_str) = serde_json::to_string(&json_out) {
                        println!("{}", json_str);
                    }
                }
                self.bits.clear();
                return;
            }

            // Display station name
            if let Some(station) = self.rds_parser.station_name()
                && self.last_station.as_ref() != Some(&station) {
                    println!("[RDS] Station: {}", station);
                    self.last_station = Some(station);
                }
            
            // Display radio text
            if let Some(text) = self.rds_parser.radio_text()
                && self.last_radiotext.as_ref() != Some(&text) {
                    println!("[RDS] Radio Text: {}", text);
                    self.last_radiotext = Some(text);
                }

            // Display metadata from Group 0A/0B (only on change, and only if we've received data)
            let info = self.rds_parser.station_info();
            let (bits, blocks, groups) = self.rds_parser.stats();
            
            // Print debug stats periodically (every ~10k bits = ~84 groups worth)
            if bits % 10000 < 200 && bits > 0 {
                debug!("[RDS-DBG] Bits: {}, Blocks: {}, Groups: {}", bits, blocks, groups);
            }
            
            // Only print PTY and DI if they changed AND we've actually received groups
            if self.rds_parser.has_data() && 
               (self.last_pty != info.program_type || self.last_di_flags != info.di_flags) {
                println!("[RDS] PTY: {} ({}) | DI: {}", 
                    info.program_type, 
                    self.rds_parser.program_type_name(),
                    info.di_flags.as_string()
                );
                self.last_pty = info.program_type;
                self.last_di_flags = info.di_flags.clone();
            }
            
            // Print flags if present
            if info.is_traffic_program || info.is_traffic_announcement {
                let mut metadata = String::new();
                if info.is_traffic_program {
                    metadata.push_str("TP ");
                }
                if info.is_traffic_announcement {
                    metadata.push_str("TA ");
                }
                if !metadata.is_empty() {
                    println!("[RDS] Flags: {}", metadata.trim());
                }
            }
            
            // Print AF list if changed
            if info.af_list != self.last_af_list && !info.af_list.is_empty() {
                let frequencies: Vec<String> = info.af_list.iter()
                    .map(|f| format!("{:.1}", (*f as f32) * 0.01 + 87.5))
                    .collect();
                println!("[RDS] Alt Frequencies: {}", frequencies.join(", "));
                self.last_af_list = info.af_list.clone();
            }

            // Display Group 1A data
            if let Some(pin) = &info.program_item {
                println!("[RDS-1A] PIN: Day {}, {:02}:{:02}", pin.day, pin.hour, pin.minute);
            }
            if let Some(lang) = info.language_code
                && let Some(lang_name) = self.rds_parser.language_name() {
                    println!("[RDS-1A] Language: {} (0x{:02X})", lang_name, lang);
                }
            if let Some(ecc) = info.extended_country_code {
                println!("[RDS-1A] ECC: 0x{:02X}", ecc);
            }
            if let Some(tmc_id) = info.tmc_id {
                println!("[RDS-1A] TMC ID: 0x{:03X}", tmc_id);
            }

            // Display Group 3A data
            if let Some(oda) = &info.oda_info {
                println!("[RDS-3A] ODA: Target={}A, App ID=0x{:04X} ({})", 
                    oda.target_group_type,
                    oda.app_id,
                    RdsParser::oda_app_name(oda.app_id)
                );
            }

            // Display Group 4A data
            if let Some(clock) = &info.clock_time {
                println!("[RDS-4A] Time: {}-{:02}-{:02} {:02}:{:02} UTC, Offset: {:+.1}h",
                    clock.year, clock.month, clock.day, clock.hour, clock.minute, clock.local_offset
                );
            }

            // Display Group 14A data
            if let Some(eon) = &info.eon_info {
                println!("[RDS-14A] EON: PI=0x{:04X}, Variant {}", eon.pi, eon.variant);
                if let Some(pty) = eon.program_type {
                    println!("[RDS-14A] EON PTY: {}", pty);
                }
                if let Some(has_link) = eon.has_linkage
                    && has_link
                        && let Some(lsn) = eon.linkage_set {
                            println!("[RDS-14A] Linkage Set: 0x{:03X}", lsn);
                        }
            }

            self.bits.clear();
        }
    }
}
