//! Complex (I/Q) sample-rate resampling utilities.

use num_complex::Complex;

const DEFAULT_TAPS: usize = 128;
const DEFAULT_PHASES: usize = 1024;

/// High-quality complex I/Q resampler based on sinc interpolation.
///
/// Input/output are complex samples at `input_rate_hz` and `output_rate_hz`.
///
/// This implementation uses a streaming polyphase-windowed-sinc filter with
/// continuous phase, tuned for SDR front-end use.
///
/// For large downsampling ratios (>2x), automatically uses multi-stage decimation
/// to improve quality and efficiency.
pub struct ComplexResampler {
    stage1_decimator: Option<IntegerDecimator>,
    stage2_resampler: SingleStageResampler,
}

impl ComplexResampler {
    pub fn new(input_rate_hz: u32, output_rate_hz: u32) -> Result<Self, String> {
        if input_rate_hz == 0 || output_rate_hz == 0 {
            return Err("Sample rates must be non-zero".to_string());
        }

        let ratio = input_rate_hz as f64 / output_rate_hz as f64;

        // For large downsampling (>2x), use multi-stage approach
        if ratio > 2.0 {
            // Find a good integer decimation factor
            let decim_factor = (ratio.sqrt().floor() as u32).max(2);
            if input_rate_hz.is_multiple_of(decim_factor) {
                let intermediate_rate = input_rate_hz / decim_factor;
                let decimator = IntegerDecimator::new(decim_factor)?;
                let resampler = SingleStageResampler::new(intermediate_rate, output_rate_hz)?;
                return Ok(Self {
                    stage1_decimator: Some(decimator),
                    stage2_resampler: resampler,
                });
            }
        }

        // Single-stage resampling
        let resampler = SingleStageResampler::new(input_rate_hz, output_rate_hz)?;
        Ok(Self {
            stage1_decimator: None,
            stage2_resampler: resampler,
        })
    }

    /// Process a chunk of complex I/Q samples.
    pub fn process(&mut self, input: &[Complex<f32>]) -> Vec<Complex<f32>> {
        if input.is_empty() {
            return Vec::new();
        }

        if let Some(decimator) = &mut self.stage1_decimator {
            let decimated = decimator.process(input);
            self.stage2_resampler.process(&decimated)
        } else {
            self.stage2_resampler.process(input)
        }
    }
}

/// Integer decimator with anti-aliasing filter
struct IntegerDecimator {
    factor: u32,
    filter_taps: Vec<f32>,
    buffer: Vec<Complex<f32>>,
}

impl IntegerDecimator {
    fn new(factor: u32) -> Result<Self, String> {
        if factor < 2 {
            return Err("Decimation factor must be >= 2".to_string());
        }

        // Design anti-aliasing FIR filter
        // Cutoff at 0.5/factor (prevent aliasing)
        let filter_len = (factor * 16).max(32) as usize;
        let cutoff = 0.5 / factor as f64 * 0.92;
        let filter_taps = build_lowpass_filter(filter_len, cutoff);

        Ok(Self {
            factor,
            filter_taps,
            buffer: Vec::with_capacity(8192),
        })
    }

    fn process(&mut self, input: &[Complex<f32>]) -> Vec<Complex<f32>> {
        self.buffer.extend_from_slice(input);

        let filter_len = self.filter_taps.len();
        let half_len = filter_len / 2;
        let mut output = Vec::with_capacity(self.buffer.len() / self.factor as usize + 1);

        let mut n = half_len;
        while n + half_len < self.buffer.len() {
            // Apply FIR filter
            let mut acc = Complex::new(0.0f32, 0.0f32);
            for (i, &coeff) in self.filter_taps.iter().enumerate() {
                acc += self.buffer[n + i - half_len] * coeff;
            }
            output.push(acc);
            n += self.factor as usize;
        }

        // Keep unconsumed samples for next call
        let consumed = (n - half_len).min(self.buffer.len());
        self.buffer.drain(..consumed);

        output
    }
}

/// Single-stage polyphase resampler
struct SingleStageResampler {
    taps: usize,
    phases: usize,
    step: f64,
    pos: f64,
    kernel: Vec<f32>,
    buffer: Vec<Complex<f32>>,
}

impl SingleStageResampler {
    fn new(input_rate_hz: u32, output_rate_hz: u32) -> Result<Self, String> {
        let taps = DEFAULT_TAPS;
        let phases = DEFAULT_PHASES;
        let step = input_rate_hz as f64 / output_rate_hz as f64;

        // Low-pass cutoff in cycles/sample (Nyquist=0.5).
        // For downsampling, cutoff scales with output/input.
        let ratio = output_rate_hz as f64 / input_rate_hz as f64;
        let cutoff = 0.5 * ratio.min(1.0) * 0.92;
        let kernel = build_polyphase_kernel(taps, phases, cutoff);

        Ok(Self {
            taps,
            phases,
            step,
            pos: taps as f64 / 2.0 - 1.0,
            kernel,
            buffer: Vec::with_capacity(8192),
        })
    }

    /// Process a chunk of complex I/Q samples.
    fn process(&mut self, input: &[Complex<f32>]) -> Vec<Complex<f32>> {
        if input.is_empty() {
            return Vec::new();
        }

        self.buffer.extend_from_slice(input);
        let mut output = Vec::with_capacity((input.len() as f64 / self.step).ceil() as usize + 8);
        let half = self.taps / 2;

        loop {
            let i = self.pos.floor() as isize;
            let start = i - (half as isize - 1);
            let end = start + self.taps as isize;

            if start < 0 || end > self.buffer.len() as isize {
                break;
            }

            let frac = self.pos - i as f64;
            let mut phase = (frac * self.phases as f64).round() as usize;
            if phase >= self.phases {
                phase = self.phases - 1;
            }

            let coeffs = &self.kernel[phase * self.taps..(phase + 1) * self.taps];
            let mut acc = Complex::new(0.0f32, 0.0f32);
            let mut idx = start as usize;
            for c in coeffs {
                acc += self.buffer[idx] * *c;
                idx += 1;
            }

            output.push(acc);
            self.pos += self.step;
        }

        // Drop consumed prefix while keeping enough history for next call.
        let keep_before = (half as isize - 1).max(0) as usize;
        let consumed = self.pos.floor().max(0.0) as usize;
        if consumed > keep_before {
            let drop_n = consumed - keep_before;
            self.buffer.drain(..drop_n);
            self.pos -= drop_n as f64;
        }

        output
    }
}

fn build_lowpass_filter(taps: usize, cutoff: f64) -> Vec<f32> {
    let center = (taps - 1) as f64 / 2.0;
    let mut filter = vec![0.0f32; taps];
    let mut sum = 0.0f64;

    for (n, f) in filter.iter_mut().enumerate() {
        let x = n as f64 - center;
        let h = 2.0 * cutoff * sinc(2.0 * cutoff * x) * blackman(n, taps);
        *f = h as f32;
        sum += h;
    }

    // Normalize
    if sum.abs() > 1e-12 {
        let norm = (1.0 / sum) as f32;
        for f in &mut filter {
            *f *= norm;
        }
    }

    filter
}

fn sinc(x: f64) -> f64 {
    if x.abs() < 1e-12 {
        1.0
    } else {
        let px = std::f64::consts::PI * x;
        px.sin() / px
    }
}

fn blackman(n: usize, len: usize) -> f64 {
    let a0 = 0.42;
    let a1 = 0.5;
    let a2 = 0.08;
    let x = 2.0 * std::f64::consts::PI * n as f64 / (len as f64 - 1.0);
    a0 - a1 * x.cos() + a2 * (2.0 * x).cos()
}

fn build_polyphase_kernel(taps: usize, phases: usize, cutoff: f64) -> Vec<f32> {
    let center = taps as f64 / 2.0 - 1.0;
    let mut table = vec![0.0f32; taps * phases];

    for p in 0..phases {
        let frac = p as f64 / phases as f64;
        let mut sum = 0.0f64;
        for n in 0..taps {
            let x = n as f64 - center - frac;
            let h = 2.0 * cutoff * sinc(2.0 * cutoff * x) * blackman(n, taps);
            table[p * taps + n] = h as f32;
            sum += h;
        }

        if sum.abs() > 1e-12 {
            let norm = (1.0 / sum) as f32;
            for n in 0..taps {
                table[p * taps + n] *= norm;
            }
        }
    }

    table
}
