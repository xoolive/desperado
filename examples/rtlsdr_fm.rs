use crossbeam::channel;
use desperado::dsp::{DspBlock, afc::SquareFreqOffsetCorrection, decimator::Decimator};
use futures::StreamExt;
use std::f32::consts::PI;
use std::io::{Write, stdout};
use std::str::FromStr;

use clap::Parser;
use desperado::IqAsyncSource;
use num_complex::Complex;

use rubato::{
    Resampler, SincFixedOut, SincInterpolationParameters, SincInterpolationType, WindowFunction,
};
use tinyaudio::prelude::*;

#[derive(Debug, Clone, Copy)]
struct Frequency(u32);

#[derive(Parser, Debug)]
#[command(author, version, about = "A waterfall plot generator from IQ files", long_about = None)]
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

    /// Frequency offset in Hz
    #[arg(short, long, default_value_t = 200_000)]
    offset_freq: i32,

    /// Enable automatic frequency correction
    #[arg(short, long, default_value_t = false)]
    afc: bool,
}

const FM_BANDWIDTH: f32 = 240_000.0; // this is a multiple of 48_000 (audio rate)
const MONO_SIGNAL_BW: f32 = 15_000.0;
const AUDIO_RATE: usize = 48_000;

#[tokio::main]
async fn main() -> tokio::io::Result<()> {
    let args = Args::parse();
    let mut iq_file = IqAsyncSource::from_rtlsdr(
        0,
        args.center_freq.0 - args.offset_freq as u32,
        args.sample_rate,
        args.gain,
    )
    .await
    .map_err(|e| std::io::Error::other(format!("{}", e)))?;

    // Setup audio output (48kHz mono)
    let (tx, rx) = channel::bounded::<f32>(AUDIO_RATE * 2); // ~2 sec buffer

    let config = OutputDeviceParameters {
        channels_count: 1,
        sample_rate: AUDIO_RATE,
        channel_sample_count: 1024,
    };
    let _device = run_output_device(config, move |data| {
        for sample in data.iter_mut() {
            *sample = rx.try_recv().unwrap_or(0.0); // fill silence if buffer underruns
        }
    })
    .unwrap();

    let mut rotate = Rotate::new(-2.0 * PI * args.offset_freq as f32 / args.sample_rate as f32);
    let mut phase_extractor = PhaseExtractor::new();
    let factor = (args.sample_rate as f32 / FM_BANDWIDTH).round() as usize;
    let mut decimator = Decimator::new(factor);

    let n = 2048; // FFT block size
    let window = 64; // search window around center
    let wide = false; // optional, only if you expect very large offsets
    let mut afc = SquareFreqOffsetCorrection::with_params(n, window, wide);

    let lowpass_fir = LowPassFir::new(MONO_SIGNAL_BW, FM_BANDWIDTH, 256);
    let mut deemphasis = DeemphasisFilter::new(FM_BANDWIDTH, 50e-6);

    let mut audio_resample =
        AudioAdaptiveResampler::new(AUDIO_RATE as f64 / FM_BANDWIDTH as f64, 1);

    // Add a simple AGC with attack/release
    let mut agc_gain = 0.5f32; // Start with moderate gain
    const AGC_ATTACK: f32 = 0.999;
    const AGC_RELEASE: f32 = 0.9999;

    while let Some(chunk) = iq_file.next().await {
        let chunk = chunk.map_err(|e| std::io::Error::other(format!("{}", e)))?;

        // 1. Frequency shift (if freq_offset != 0)
        let shifted = rotate.process(&chunk);

        // 2. Downsample to FM bandwidth
        let decimated = decimator.process(&shifted);

        // 2.b. Automatic Frequency Correction
        let afc_corrected = if args.afc {
            afc.process(&decimated)
        } else {
            decimated
        };

        // 3. Phase extraction for FM demodulation
        let phase = phase_extractor.process(&afc_corrected);

        // 4. Lowpass filtering
        let filtered = lowpass_fir.process(&phase);

        // 5. Deemphasis filtering
        let deemphasized = deemphasis.process(&filtered);

        //total_input_samples += deemphasized.len();
        // --- Adaptive resampling to 48kHz ---
        // Rubato expects Vec<Vec<f32>> where outer vec is channels

        let audio = audio_resample.process(&deemphasized);

        // Now process AGC and send entire accumulated buffer at once
        //if !accumulated_audio.is_empty() {
        // Update AGC gain based on entire buffer
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

        // Apply AGC and soft clipping to entire buffer
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

        audio_resample.adjust_ratio(tx.len() as f64 / (AUDIO_RATE * 2) as f64);

        // If buffer is very full, skip this chunk entirely
        if tx.len() > (AUDIO_RATE * 2) * 9 / 10 {
            println!(
                "\n[DROP] Buffer >90% full, dropping {} samples",
                processed.len()
            );
            continue; // Skip to next IQ chunk
        }

        // Send entire buffer at once
        for sample in processed {
            if tx.try_send(sample).is_err() {
                println!("\n[WARNING] Audio buffer full");
                break;
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

pub struct Rotate {
    /// Current rotation phasor.
    rot: Complex<f32>,
    /// Rotation step multiplier.
    mult: Complex<f32>,
}

impl Rotate {
    pub fn new(angle: f32) -> Self {
        Self {
            rot: Complex::new(1.0, 0.0),
            mult: Complex::new(angle.cos(), angle.sin()),
        }
    }

    pub fn process(&mut self, data: &[Complex<f32>]) -> Vec<Complex<f32>> {
        let mut output_up = Vec::with_capacity(data.len());

        for &sample in data {
            let rr = sample.re * self.rot.re;
            let ii = sample.im * self.rot.im;
            let ri = sample.re * self.rot.im;
            let ir = sample.im * self.rot.re;

            output_up.push(Complex::new(rr - ii, ir + ri));

            // Update rotation phasor
            self.rot *= self.mult;
        }

        // Normalize to prevent drift
        let norm = self.rot.norm();
        if norm > 0.0 {
            self.rot /= norm;
        }

        output_up
    }
}

struct PhaseExtractor {
    last: Complex<f32>,
}

impl PhaseExtractor {
    fn new() -> Self {
        Self {
            last: Complex::<f32>::new(1.0, 0.0),
        }
    }

    fn process(&mut self, samples: &[Complex<f32>]) -> Vec<f32> {
        let mut phases = Vec::with_capacity(samples.len());
        for &sample in samples {
            let d = (sample * self.last.conj()).arg();
            phases.push(d);
            self.last = sample;
        }
        // Normalize to [-1, 1]
        let max_val = phases.iter().fold(0.0_f32, |a, &b| a.max(b.abs()));
        if max_val > 0.0 {
            for p in phases.iter_mut() {
                *p /= max_val;
            }
        }
        phases
    }
}

struct LowPassFir {
    fir: Vec<f32>,
}

impl LowPassFir {
    fn new(cutoff_freq: f32, sample_rate: f32, taps: usize) -> Self {
        let mut fir = Vec::with_capacity(taps);
        let mid = (taps / 2) as isize;
        let norm_cutoff = cutoff_freq / (sample_rate / 2.0);

        // Calculate FIR coefficients using the Remez exchange algorithm (Parks-McClellan).
        // For now, we use a windowed-sinc as a placeholder,
        // but you can use the reez for sharper filters.
        //
        // let bands = vec![
        //     Band::new(0.0, norm_cutoff, 1.0), // Passband
        //     Band::new(norm_cutoff + 0.05, 0.5, 0.0), // Stopband
        // ];
        // let coeffs = remez(taps, &bands, FilterType::LowPass).unwrap();
        // fir.extend(coeffs);
        //
        // For now, fallback to Blackman-windowed sinc:
        for n in 0..taps {
            let x = n as isize - mid;
            let sinc = if x == 0 {
                2.0 * norm_cutoff
            } else {
                (2.0 * norm_cutoff * PI * x as f32).sin() / (PI * x as f32)
            };
            let window = 0.42 - 0.5 * ((2.0 * PI * n as f32) / (taps as f32 - 1.0)).cos()
                + 0.08 * ((4.0 * PI * n as f32) / (taps as f32 - 1.0)).cos(); // Blackman
            fir.push(sinc * window);
        }
        let norm: f32 = fir.iter().sum();
        for v in fir.iter_mut() {
            *v /= norm;
        }
        Self { fir }
    }

    fn process(&self, samples: &[f32]) -> Vec<f32> {
        let taps = self.fir.len();
        let mid = taps / 2;
        let mut out = vec![0.0f32; samples.len()];
        for (i, out_elem) in out.iter_mut().enumerate() {
            let mut acc = 0.0f32;
            for j in 0..taps {
                let idx = i as isize + j as isize - mid as isize;
                if idx >= 0 && (idx as usize) < samples.len() {
                    acc += samples[idx as usize] * self.fir[j];
                }
            }
            *out_elem = acc;
        }
        out
    }
}

struct DeemphasisFilter {
    a: f32,
    b: f32,
    prev_y: f32,
}

impl DeemphasisFilter {
    fn new(sample_rate: f32, tau: f32) -> Self {
        let dt = 1.0 / sample_rate;
        let decay = (-dt / tau).exp();
        let b = 1.0 - decay;
        let a = decay;
        Self { a, b, prev_y: 0.0 }
    }

    fn process(&mut self, samples: &[f32]) -> Vec<f32> {
        let mut y = Vec::with_capacity(samples.len());
        for &x in samples {
            let out = self.b * x + self.a * self.prev_y;
            y.push(out);
            self.prev_y = out;
        }
        y
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
}

impl AudioAdaptiveResampler {
    fn new(initial_ratio: f64, adjustment_interval: usize) -> Self {
        let params = SincInterpolationParameters {
            sinc_len: 256,
            f_cutoff: 0.95,
            interpolation: SincInterpolationType::Cubic,
            oversampling_factor: 160,
            window: WindowFunction::BlackmanHarris2,
        };

        let output_frames = 1024; // Fixed output size per call

        let resampler = SincFixedOut::<f32>::new(
            initial_ratio,
            2.0, // max_resample_ratio_relative
            params,
            output_frames,
            1, // number of channels
        )
        .unwrap();

        Self {
            resampler,
            target_fill: 0.3,
            alpha: 0.95,
            k_p: 0.0001,
            k_i: 1e-7,
            smoothed_error: 0.0,
            integral_error: 0.0,
            adjustment_interval,
            adjustment_counter: 0,
            leftover: Vec::new(),
            resample_ratio: initial_ratio,
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

            // Only accumulate integral when error is small (anti-windup)
            if self.smoothed_error.abs() < 0.15 {
                self.integral_error += self.smoothed_error;
                // Clamp integral to prevent runaway
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

    fn process(&mut self, input: &[f32]) -> Vec<f32> {
        // Append new input to leftover buffer
        self.leftover.extend_from_slice(input);

        let mut output = Vec::new();

        // Process as many complete blocks as possible
        loop {
            // SincFixedOut requires EXACTLY the number of input frames it asks for
            let input_needed = self.resampler.input_frames_next();

            if self.leftover.len() < input_needed {
                // Not enough input samples yet, wait for next call
                break;
            }

            // Extract exactly the needed number of samples
            let input_chunk: Vec<f32> = self.leftover.drain(..input_needed).collect();

            // Wrap in Vec<Vec<f32>> for channel format (1 channel)
            let input_block = vec![input_chunk];

            // Process - this will produce exactly output_frames samples
            match self.resampler.process(&input_block, None) {
                Ok(output_block) => {
                    output.extend_from_slice(&output_block[0]);
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
