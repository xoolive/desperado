use crossbeam::channel;
use futures::StreamExt;
use std::f32::consts::PI;
use std::str::FromStr;

use clap::Parser;
use desperado::IqAsyncSource;
use desperado::dsp::{
    DspBlock, afc::SquareFreqOffsetCorrection, decimator::Decimator, rds::RdsParser, rotate::Rotate,
};
use num_complex::Complex;
use rubato::{
    Resampler, SincFixedOut, SincInterpolationParameters, SincInterpolationType, WindowFunction,
};
use tinyaudio::prelude::*;

/// Command-line arguments
#[derive(Parser, Debug)]
#[command(author, version, about = "Stereo FM demodulator from RTL-SDR", long_about = None)]
struct Args {
    #[arg(short, long, value_parser = Frequency::from_str)]
    center_freq: Frequency,

    #[arg(short, long, default_value_t = 2_000_000)]
    sample_rate: u32,

    #[arg(short, long, default_value = None)]
    gain: Option<i32>,

    #[arg(short, long, default_value_t = -200_000)]
    offset_freq: i32,

    #[arg(short, long, default_value_t = false)]
    afc: bool,
}

#[derive(Debug, Clone, Copy)]
struct Frequency(u32);

impl FromStr for Frequency {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let s = s.trim();
        if let Some(stripped) = s.strip_suffix('M') {
            let val: f32 = stripped.parse().map_err(|_| "Invalid MHz value")?;
            Ok(Frequency((val * 1_000_000.0) as u32))
        } else if let Some(stripped) = s.strip_suffix('k') {
            let val: f32 = stripped.parse().map_err(|_| "Invalid kHz value")?;
            Ok(Frequency((val * 1_000.0) as u32))
        } else {
            let val: u32 = s.parse().map_err(|_| "Invalid Hz value")?;
            Ok(Frequency(val))
        }
    }
}

const FM_BANDWIDTH: f32 = 240_000.0;
const MONO_SIGNAL_BW: f32 = 15_000.0;
const AUDIO_RATE: usize = 48_000;

#[tokio::main]
async fn main() -> tokio::io::Result<()> {
    let args = Args::parse();

    /*let mut source = IqAsyncSource::from_rtlsdr(
        0,
        args.center_freq.0 - args.offset_freq as u32,
        args.sample_rate,
        args.gain,
    )
    .await
    .map_err(|e| std::io::Error::other(format!("{}", e)))?;*/

    let mut source = IqAsyncSource::from_soapysdr(
        "rtlsdr",
        0,
        (args.center_freq.0 as i32 - args.offset_freq) as u32,
        args.sample_rate,
        args.gain.map(|g| g as f64),
        "TUNER",
    )
    .await
    .map_err(|e| std::io::Error::other(format!("{}", e)))?;

    /*let mut source = IqAsyncSource::from_file(
        "/Users/xo/Documents/isae/pybook/python_web_github/data/samples.rtl",
        args.center_freq.0 - args.offset_freq as u32,
        args.sample_rate,
        16_384,
        desperado::IqFormat::Cu8,
    )
    .await
    .map_err(|e| std::io::Error::other(format!("{}", e)))?;*/

    // --- Stereo audio output ---
    let (tx, rx) = channel::bounded::<f32>(AUDIO_RATE * 2); // Reduce buffer to 1 sec
    let config = OutputDeviceParameters {
        channels_count: 2,
        sample_rate: AUDIO_RATE,
        channel_sample_count: 1024,
    };
    let _device = run_output_device(config, move |data| {
        for sample in data.iter_mut() {
            *sample = rx.try_recv().unwrap_or(0.0);
        }
    })
    .unwrap();

    let mut rotate = Rotate::new(-2.0 * PI * args.offset_freq as f32 / args.sample_rate as f32);
    let mut phase_extractor = PhaseExtractor::new();
    let factor = (args.sample_rate as f32 / FM_BANDWIDTH).round() as usize;
    let mut decimator = Decimator::new(factor);

    let n = 2048;
    let window = 64;
    let mut afc = SquareFreqOffsetCorrection::with_params(n, window, false);
    let lowpass_fir = LowPassFir::new(MONO_SIGNAL_BW, FM_BANDWIDTH, 256);
    let mut stereo = StereoDecoderPLL::new(FM_BANDWIDTH);
    let mut deemphasis_l = DeemphasisFilter::new(FM_BANDWIDTH, 50e-6);
    let mut deemphasis_r = DeemphasisFilter::new(FM_BANDWIDTH, 50e-6);
    let mut rds = RdsDecoder::new(FM_BANDWIDTH);

    let mut audio_resample =
        StereoAudioAdaptiveResampler::new(AUDIO_RATE as f64 / FM_BANDWIDTH as f64, 5, 2); // More frequent adjustments

    println!("Running stereo FM demodulator...");

    let t0 = std::time::Instant::now();
    let mut total_samples = 0u64;

    while let Some(chunk) = source.next().await {
        let chunk = chunk.map_err(|e| std::io::Error::other(format!("{}", e)))?;
        let chunk_samples = chunk.len() as u64;
        total_samples += chunk_samples;

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

        // Resample and adapt
        let audio = audio_resample.process(&interleaved);
        // Calculate buffer fill ratio (0.0 = empty, 1.0 = full)
        //let buffer_fill = tx.len() as f64 / (AUDIO_RATE * 2) as f64;
        //audio_resample.adjust_ratio(buffer_fill);

        // Send to audio
        for sample in audio {
            if tx.try_send(sample).is_err() {
                // Drop samples if buffer is full instead of breaking
                break;
            }
        }

        // --- simulate real-time pacing ---
        // only for file input source
        if let IqAsyncSource::IqAsyncFile(_) = source {
            let expected_time = total_samples as f64 / args.sample_rate as f64;
            let elapsed = t0.elapsed().as_secs_f64();
            if expected_time > elapsed {
                let sleep_dur = expected_time - elapsed;
                tokio::time::sleep(std::time::Duration::from_secs_f64(sleep_dur)).await;
            }
        }
    }

    Ok(())
}

// --- PhaseExtractor ---
struct PhaseExtractor {
    last: Complex<f32>,
}
impl PhaseExtractor {
    fn new() -> Self {
        Self {
            last: Complex::new(1.0, 0.0),
        }
    }
    fn process(&mut self, s: &[Complex<f32>]) -> Vec<f32> {
        let mut out = Vec::with_capacity(s.len());
        for &x in s {
            let d = (x * self.last.conj()).arg();
            out.push(d);
            self.last = x;
        }
        out
    }
}

// --- LowPassFir ---
struct LowPassFir {
    fir: Vec<f32>,
}
impl LowPassFir {
    fn new(cutoff_freq: f32, sample_rate: f32, taps: usize) -> Self {
        let mut fir = Vec::with_capacity(taps);
        let mid = (taps / 2) as isize;
        let norm_cutoff = cutoff_freq / (sample_rate / 2.0);
        for n in 0..taps {
            let x = n as isize - mid;
            let sinc = if x == 0 {
                2.0 * norm_cutoff
            } else {
                (2.0 * norm_cutoff * PI * x as f32).sin() / (PI * x as f32)
            };
            let win = 0.42 - 0.5 * ((2.0 * PI * n as f32) / (taps as f32 - 1.0)).cos()
                + 0.08 * ((4.0 * PI * n as f32) / (taps as f32 - 1.0)).cos();
            fir.push(sinc * win);
        }
        let norm: f32 = fir.iter().sum();
        for v in fir.iter_mut() {
            *v /= norm;
        }
        Self { fir }
    }
    fn process(&self, s: &[f32]) -> Vec<f32> {
        let taps = self.fir.len();
        let mid = taps / 2;
        let mut out = vec![0.0; s.len()];
        for (i, o) in out.iter_mut().enumerate() {
            let mut acc = 0.0;
            for j in 0..taps {
                let idx = i as isize + j as isize - mid as isize;
                if idx >= 0 && (idx as usize) < s.len() {
                    acc += s[idx as usize] * self.fir[j];
                }
            }
            *o = acc;
        }
        out
    }
}

// --- Deemphasis ---
struct DeemphasisFilter {
    a: f32,
    b: f32,
    prev_y: f32,
}
impl DeemphasisFilter {
    fn new(rate: f32, tau: f32) -> Self {
        let dt = 1.0 / rate;
        let decay = (-dt / tau).exp();
        let b = 1.0 - decay;
        let a = decay;
        Self { a, b, prev_y: 0.0 }
    }
    fn process(&mut self, s: &[f32]) -> Vec<f32> {
        let mut y = Vec::with_capacity(s.len());
        for &x in s {
            let out = self.b * x + self.a * self.prev_y;
            y.push(out);
            self.prev_y = out;
        }
        y
    }
}

// --- StereoDecoderPLL and StereoAudioAdaptiveResampler ---
// Use the exact implementations we discussed earlier.
/// Stereo decoder using a complex PLL locked to the 19 kHz pilot tone.
/// Regenerates 38 kHz carrier to demodulate L−R DSBSC component.
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

/// Stereo (interleaved) adaptive resampler using rubato::SincFixedOut.
/// Accepts interleaved stereo input (L,R,L,R,...) and outputs interleaved stereo output.
pub struct StereoAudioAdaptiveResampler {
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

impl StereoAudioAdaptiveResampler {
    pub fn new(initial_ratio: f64, adjustment_interval: usize, channels: usize) -> Self {
        let params = SincInterpolationParameters {
            sinc_len: 256,
            f_cutoff: 0.95,
            interpolation: SincInterpolationType::Cubic,
            oversampling_factor: 160,
            window: WindowFunction::BlackmanHarris2,
        };

        let resampler = SincFixedOut::<f32>::new(initial_ratio, 1.02, params, 1024, channels)
            .expect("failed to create resampler");

        Self {
            resampler,
            target_fill: 0.4, // Target 40% fill instead of 50%
            alpha: 0.9,       // More responsive smoothing
            k_p: 0.002,       // More aggressive proportional gain
            k_i: 5e-6,        // More aggressive integral gain
            smoothed_error: 0.0,
            integral_error: 0.0,
            adjustment_interval,
            adjustment_counter: 0,
            leftover: Vec::new(),
            resample_ratio: initial_ratio,
            channels,
        }
    }

    pub fn adjust_ratio(&mut self, buffer_fill: f64) {
        self.adjustment_counter += 1;
        if self.adjustment_counter < self.adjustment_interval {
            return;
        }
        self.adjustment_counter = 0;

        let error = self.target_fill - buffer_fill;
        self.smoothed_error = self.alpha * self.smoothed_error + (1.0 - self.alpha) * error;

        if self.smoothed_error.abs() < 0.005 {
            // Tighter deadband
            self.smoothed_error = 0.0;
        }

        if self.smoothed_error.abs() < 0.3 {
            // Wider integration range
            self.integral_error += self.smoothed_error;
            self.integral_error = self.integral_error.clamp(-1e5, 1e5); // Tighter clamp
        }

        // Positive error means buffer is emptier than target, so we need to resample slower (produce more samples)
        // Negative error means buffer is fuller than target, so we need to resample faster (produce fewer samples)
        self.resample_ratio += self.k_p * self.smoothed_error + self.k_i * self.integral_error;

        let nominal = AUDIO_RATE as f64 / FM_BANDWIDTH as f64;
        let max_rel = 1.02;
        let min_ratio = nominal / max_rel;
        let max_ratio = nominal * max_rel;
        self.resample_ratio = self.resample_ratio.clamp(min_ratio, max_ratio);

        if let Err(e) =
            Resampler::set_resample_ratio(&mut self.resampler, self.resample_ratio, true)
        {
            eprintln!("Resampler ratio update failed: {:?}", e);
        }
    }

    pub fn process(&mut self, interleaved_input: &[f32]) -> Vec<f32> {
        self.leftover.extend_from_slice(interleaved_input);
        let mut output_interleaved = Vec::new();

        loop {
            let input_frames_needed = self.resampler.input_frames_next();
            let samples_needed = input_frames_needed * self.channels;
            if self.leftover.len() < samples_needed {
                break;
            }

            let chunk: Vec<f32> = self.leftover.drain(..samples_needed).collect();

            let mut chs: Vec<Vec<f32>> =
                vec![Vec::with_capacity(input_frames_needed); self.channels];
            for frame_idx in 0..input_frames_needed {
                (0..self.channels).for_each(|ch| {
                    let idx = frame_idx * self.channels + ch;
                    chs[ch].push(chunk[idx]);
                });
            }

            if let Ok(output_blocks) = Resampler::process(&mut self.resampler, &chs, None) {
                let out_frames = output_blocks[0].len();
                (0..out_frames).for_each(|i| {
                    (0..self.channels).for_each(|ch| {
                        output_interleaved.push(output_blocks[ch][i]);
                    });
                });
            } else {
                break;
            }
        }

        output_interleaved
    }
}

/// Simple RDS (Radio Data System) decoder.
/// Extracts 57 kHz subcarrier, BPSK demodulates, and symbol-slices it.
pub struct RdsDecoder {
    sample_rate: f32,
    // Costas loop for carrier recovery
    pll_phase: f64,
    pll_freq: f64,
    nominal_freq: f64,
    pll_alpha: f64,
    pll_beta: f64,
    // Filters
    bpf_hi: LowPassFir,
    bpf_lo: LowPassFir,
    lpf: LowPassFir,
    // Symbol timing recovery
    #[allow(dead_code)]
    symbol_rate: f32,
    symbol_phase: f32,
    symbol_period: f32,
    ted_gain: f32,
    // Differential decoder state
    last_bit: u8,
    bits: Vec<u8>,
    rds_parser: RdsParser,
    last_station: Option<String>,
    last_radiotext: Option<String>,
}

impl RdsDecoder {
    pub fn new(sample_rate: f32) -> Self {
        let nominal = 57_000.0_f64;
        Self {
            sample_rate,
            pll_phase: 0.0,
            pll_freq: nominal,
            nominal_freq: nominal,
            pll_alpha: 0.01, // Loop filter gains for Costas loop
            pll_beta: 0.00001,
            // Bandpass around 57 kHz using high-pass and low-pass
            bpf_hi: LowPassFir::new(60_000.0, sample_rate, 257),
            bpf_lo: LowPassFir::new(54_000.0, sample_rate, 257),
            lpf: LowPassFir::new(2_400.0, sample_rate, 257),
            symbol_rate: 1187.5,
            symbol_phase: 0.0,
            symbol_period: sample_rate / 1187.5,
            ted_gain: 0.01,
            last_bit: 0,
            bits: Vec::new(),
            rds_parser: RdsParser::new(),
            last_station: None,
            last_radiotext: None,
        }
    }

    /// Process one FM baseband block
    pub fn process(&mut self, input: &[f32]) {
        // --- 1. Bandpass filter around 57 kHz ---
        let hi = self.bpf_hi.process(input);
        let lo = self.bpf_lo.process(input);
        let mut bandpass = Vec::with_capacity(input.len());
        for i in 0..input.len() {
            bandpass.push(hi[i] - lo[i]);
        }

        // --- 2. Costas loop for carrier recovery (BPSK) ---
        let mut demod = Vec::with_capacity(bandpass.len());
        for &x in &bandpass {
            // Mix with carrier
            let cos_phase = (2.0 * std::f64::consts::PI * self.pll_phase).cos() as f32;
            let sin_phase = (2.0 * std::f64::consts::PI * self.pll_phase).sin() as f32;

            let i_mix = x * cos_phase;
            let q_mix = x * sin_phase;

            demod.push(i_mix);

            // Costas loop error detector for BPSK: error = sign(I) * Q
            let error = i_mix.signum() * q_mix;

            // Update PLL
            self.pll_freq += self.pll_beta * error as f64;
            self.pll_freq = self
                .pll_freq
                .clamp(self.nominal_freq * 0.98, self.nominal_freq * 1.02);

            self.pll_phase +=
                self.pll_freq / self.sample_rate as f64 + self.pll_alpha * error as f64;

            // Wrap phase
            while self.pll_phase >= 1.0 {
                self.pll_phase -= 1.0;
            }
            while self.pll_phase < 0.0 {
                self.pll_phase += 1.0;
            }
        }

        // --- 3. Lowpass filter ---
        let filtered = self.lpf.process(&demod);

        // --- 4. Symbol timing recovery with Gardner TED ---
        let mut symbols = Vec::new();
        let mut i = 0;
        while i < filtered.len() {
            let sample_idx = self.symbol_phase as usize;
            if sample_idx + 2 < filtered.len() {
                // Sample at symbol time
                let mid = filtered[sample_idx + 1];
                symbols.push(mid);

                // Gardner timing error detector
                if sample_idx > 0 && sample_idx + 2 < filtered.len() {
                    let early = filtered[sample_idx];
                    let late = filtered[sample_idx + 2];
                    let error = (late - early) * mid;

                    // Adjust symbol phase
                    self.symbol_phase += self.symbol_period - self.ted_gain * error;
                } else {
                    self.symbol_phase += self.symbol_period;
                }
            } else {
                break;
            }

            i = self.symbol_phase as usize;
        }

        // Reset phase for next block
        self.symbol_phase -= filtered.len() as f32;
        if self.symbol_phase < 0.0 {
            self.symbol_phase += self.symbol_period;
        }

        // --- 5. Differential Manchester decoding ---
        for &symbol in &symbols {
            let bit = if symbol > 0.0 { 1 } else { 0 };
            // Differential decoding: XOR with previous bit
            let decoded = bit ^ self.last_bit;
            self.bits.push(decoded);
            self.last_bit = bit;
        }

        // --- 6. Feed to RDS parser ---
        if self.bits.len() >= 104 {
            self.rds_parser.push_bits(&self.bits);

            // Check for updates (print only when changed)
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
