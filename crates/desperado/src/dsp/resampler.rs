//! Complex (I/Q) sample-rate resampling utilities.

use crate::dsp::buffer::StreamBuffer;
use crate::dsp::window::{WindowType, compute_window_f64, sinc};
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
    /// Create a new complex I/Q resampler.
    ///
    /// For ratios > 2×, automatically uses a two-stage approach (integer
    /// decimation followed by polyphase sinc) for better quality.
    ///
    /// # Arguments
    /// * `input_rate_hz` - Input sample rate in Hz
    /// * `output_rate_hz` - Desired output sample rate in Hz
    ///
    /// # Errors
    /// Returns an error if either rate is zero.
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
    buffer: StreamBuffer<Complex<f32>>,
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
            buffer: StreamBuffer::with_capacity(8192),
        })
    }

    fn process(&mut self, input: &[Complex<f32>]) -> Vec<Complex<f32>> {
        self.buffer.push_slice(input);

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
        self.buffer.consume(consumed);

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
    buffer: StreamBuffer<Complex<f32>>,
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
            buffer: StreamBuffer::with_capacity(8192),
        })
    }

    /// Process a chunk of complex I/Q samples.
    fn process(&mut self, input: &[Complex<f32>]) -> Vec<Complex<f32>> {
        if input.is_empty() {
            return Vec::new();
        }

        self.buffer.push_slice(input);
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
            for (idx, c) in (start as usize..).zip(coeffs.iter()) {
                acc += self.buffer[idx] * *c;
            }

            output.push(acc);
            self.pos += self.step;
        }

        // Drop consumed prefix while keeping enough history for next call.
        let keep_before = (half as isize - 1).max(0) as usize;
        let consumed = self.pos.floor().max(0.0) as usize;
        if consumed > keep_before {
            let drop_n = consumed - keep_before;
            self.buffer.consume(drop_n);
            self.pos -= drop_n as f64;
        }

        output
    }
}

/// Build an anti-aliasing low-pass FIR filter using a Blackman-windowed sinc.
///
/// Uses f64 precision for coefficient computation, then casts to f32.
fn build_lowpass_filter(taps: usize, cutoff: f64) -> Vec<f32> {
    let center = (taps - 1) as f64 / 2.0;
    let mut filter = vec![0.0f32; taps];
    let mut sum = 0.0f64;

    for (n, f) in filter.iter_mut().enumerate() {
        let x = n as f64 - center;
        let h = 2.0
            * cutoff
            * sinc(2.0 * cutoff * x)
            * compute_window_f64(n, taps, WindowType::Blackman);
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

/// Build a polyphase sinc kernel for fractional-delay interpolation.
///
/// Generates `phases × taps` coefficients. Each phase is a Blackman-windowed
/// sinc filter shifted by `phase / phases` of a sample, normalised to unity gain.
fn build_polyphase_kernel(taps: usize, phases: usize, cutoff: f64) -> Vec<f32> {
    let center = taps as f64 / 2.0 - 1.0;
    let mut table = vec![0.0f32; taps * phases];

    for p in 0..phases {
        let frac = p as f64 / phases as f64;
        let mut sum = 0.0f64;
        for n in 0..taps {
            let x = n as f64 - center - frac;
            let h = 2.0
                * cutoff
                * sinc(2.0 * cutoff * x)
                * compute_window_f64(n, taps, WindowType::Blackman);
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

/// Stateful 2:1 averaging decimator for complex samples.
///
/// Produces one output sample for every two input samples by averaging
/// adjacent pairs. Maintains state across `process()` calls so that an
/// odd trailing sample is paired with the first sample of the next block.
///
/// This is a cheap alternative to a full FIR decimator when the input is
/// already bandwidth-limited (e.g. hardware anti-alias filter in the ADC).
struct AveragingDecimator2 {
    pending: Option<Complex<f32>>,
}

impl AveragingDecimator2 {
    fn new() -> Self {
        Self { pending: None }
    }

    fn process(&mut self, input: &[Complex<f32>]) -> Vec<Complex<f32>> {
        if input.is_empty() {
            return Vec::new();
        }

        let mut output = Vec::with_capacity(input.len().div_ceil(2));
        let mut idx = 0usize;

        if let Some(prev) = self.pending.take() {
            let pair_avg = (prev + input[0]) * 0.5;
            output.push(pair_avg);
            idx = 1;
        }

        while idx + 1 < input.len() {
            let pair_avg = (input[idx] + input[idx + 1]) * 0.5;
            output.push(pair_avg);
            idx += 2;
        }

        if idx < input.len() {
            self.pending = Some(input[idx]);
        }

        output
    }
}

/// Multi-stage complex resampler that picks an optimal processing path
/// based on the input and output sample rates.
///
/// For common SDR front-end rates, this selects efficient combinations of
/// 2:1 averaging decimation and polyphase sinc resampling to minimise
/// computation while maintaining signal quality.
///
/// # Supported paths
///
/// | Input rate          | Strategy                                         |
/// |---------------------|--------------------------------------------------|
/// | Equal to output     | Passthrough (zero-copy)                          |
/// | ~2× output          | Single 2:1 averaging decimation                  |
/// | ~3× output          | Polyphase resample → 2:1 decimation              |
/// | ~1.2× output        | Direct polyphase resample                        |
/// | ≥2.8× output        | 2:1 decimate → polyphase resample → 2:1 decimate |
/// | Other               | Direct polyphase resample (fallback)             |
///
/// # Example
///
/// ```ignore
/// use desperado::dsp::resampler::MultiStageResampler;
/// use num_complex::Complex;
///
/// // Airspy 4.096 MHz → DAB native 2.048 MHz
/// let mut r = MultiStageResampler::new(4_096_000, 2_048_000).unwrap();
/// let input = vec![Complex::new(1.0, 0.0); 4096];
/// let output = r.process(&input);
/// ```
pub struct MultiStageResampler {
    path: MultiStagePath,
}

enum MultiStagePath {
    Passthrough,
    Half(AveragingDecimator2),
    Direct(ComplexResampler),
    ResampleThenHalf {
        resampler: ComplexResampler,
        decimator: AveragingDecimator2,
    },
    HalfThenResampleThenHalf {
        pre_decimator: AveragingDecimator2,
        resampler: ComplexResampler,
        post_decimator: AveragingDecimator2,
    },
}

impl MultiStageResampler {
    /// Create a multi-stage resampler for the given input and output rates.
    ///
    /// # Arguments
    /// * `input_rate_hz`  – Source sample rate in Hz
    /// * `output_rate_hz` – Desired output sample rate in Hz
    ///
    /// # Errors
    /// Returns an error if either rate is zero or the rate combination is
    /// unsupported.
    pub fn new(input_rate_hz: u32, output_rate_hz: u32) -> Result<Self, String> {
        if input_rate_hz == 0 || output_rate_hz == 0 {
            return Err("Sample rates must be non-zero".to_string());
        }

        let intermediate_2x = output_rate_hz.saturating_mul(2);

        let path = if input_rate_hz == output_rate_hz {
            MultiStagePath::Passthrough
        } else if is_near(input_rate_hz, intermediate_2x, 0.05) {
            // ~2× output → simple 2:1 averaging
            MultiStagePath::Half(AveragingDecimator2::new())
        } else if is_near(input_rate_hz, output_rate_hz.saturating_mul(3), 0.05) {
            // ~3× output → resample to 2× then halve
            MultiStagePath::ResampleThenHalf {
                resampler: ComplexResampler::new(input_rate_hz, intermediate_2x)?,
                decimator: AveragingDecimator2::new(),
            }
        } else if input_rate_hz as f64 / output_rate_hz as f64 >= 2.8 {
            // High ratio → halve, resample to 2×, halve
            let half_rate = input_rate_hz / 2;
            MultiStagePath::HalfThenResampleThenHalf {
                pre_decimator: AveragingDecimator2::new(),
                resampler: ComplexResampler::new(half_rate, intermediate_2x)?,
                post_decimator: AveragingDecimator2::new(),
            }
        } else {
            // Moderate ratio → direct polyphase resample
            MultiStagePath::Direct(ComplexResampler::new(input_rate_hz, output_rate_hz)?)
        };

        Ok(Self { path })
    }

    /// Resample a chunk of complex I/Q samples.
    pub fn process(&mut self, input: &[Complex<f32>]) -> Vec<Complex<f32>> {
        if input.is_empty() {
            return Vec::new();
        }

        match &mut self.path {
            MultiStagePath::Passthrough => input.to_vec(),
            MultiStagePath::Half(decimator) => decimator.process(input),
            MultiStagePath::Direct(resampler) => resampler.process(input),
            MultiStagePath::ResampleThenHalf {
                resampler,
                decimator,
            } => {
                let resampled = resampler.process(input);
                decimator.process(&resampled)
            }
            MultiStagePath::HalfThenResampleThenHalf {
                pre_decimator,
                resampler,
                post_decimator,
            } => {
                let half = pre_decimator.process(input);
                let intermediate = resampler.process(&half);
                post_decimator.process(&intermediate)
            }
        }
    }
}

/// Check whether `rate` is within `tolerance` (fraction) of `target`.
fn is_near(rate: u32, target: u32, tolerance: f64) -> bool {
    let ratio = rate as f64 / target as f64;
    (ratio - 1.0).abs() < tolerance
}

#[cfg(test)]
mod tests {
    use super::*;

    // -- ComplexResampler -------------------------------------------------

    #[test]
    fn test_complex_resampler_zero_rate() {
        assert!(ComplexResampler::new(0, 48000).is_err());
        assert!(ComplexResampler::new(48000, 0).is_err());
    }

    #[test]
    fn test_complex_resampler_identity() {
        let mut r = ComplexResampler::new(48000, 48000).unwrap();
        let input = vec![Complex::new(1.0, 0.0); 480];
        let output = r.process(&input);
        // Same rate: output length should be close to input length
        // (first call loses samples to filter warmup)
        assert!(
            output.len().abs_diff(480) < 150,
            "Expected ~480, got {}",
            output.len()
        );
    }

    #[test]
    fn test_complex_resampler_downsample() {
        // 2.4 MHz → 2.048 MHz (~0.85× ratio)
        let mut r = ComplexResampler::new(2_400_000, 2_048_000).unwrap();
        let input = vec![Complex::new(1.0, 0.0); 2400];
        let output = r.process(&input);
        let expected = (2400.0 * 2_048_000.0 / 2_400_000.0) as usize;
        assert!(
            output.len().abs_diff(expected) < 200,
            "Expected ~{}, got {}",
            expected,
            output.len()
        );
    }

    #[test]
    fn test_complex_resampler_empty() {
        let mut r = ComplexResampler::new(48000, 44100).unwrap();
        assert!(r.process(&[]).is_empty());
    }

    // -- MultiStageResampler ----------------------------------------------

    #[test]
    fn test_multistage_passthrough() {
        let mut r = MultiStageResampler::new(2_048_000, 2_048_000).unwrap();
        let input = vec![Complex::new(1.0, 0.0); 1024];
        let output = r.process(&input);
        assert_eq!(output.len(), 1024);
    }

    #[test]
    fn test_multistage_half() {
        // 4.096 MHz → 2.048 MHz (2:1 averaging)
        let mut r = MultiStageResampler::new(4_096_000, 2_048_000).unwrap();
        let input = vec![Complex::new(1.0, 0.0); 1024];
        let output = r.process(&input);
        assert_eq!(output.len(), 512);
    }

    #[test]
    fn test_multistage_direct() {
        // 2.4 MHz → 2.048 MHz (polyphase resample)
        let mut r = MultiStageResampler::new(2_400_000, 2_048_000).unwrap();
        let input = vec![Complex::new(1.0, 0.0); 2400];
        let output = r.process(&input);
        let expected = (2400.0 * 2_048_000.0 / 2_400_000.0) as usize;
        assert!(
            output.len().abs_diff(expected) < 200,
            "Expected ~{}, got {}",
            expected,
            output.len()
        );
    }

    #[test]
    fn test_multistage_high_ratio() {
        // 6.144 MHz → 2.048 MHz (3:1 via multi-stage)
        let mut r = MultiStageResampler::new(6_144_000, 2_048_000).unwrap();
        let input = vec![Complex::new(1.0, 0.0); 6144];
        let output = r.process(&input);
        let expected = (6144.0 * 2_048_000.0 / 6_144_000.0) as usize;
        assert!(
            output.len().abs_diff(expected) < 300,
            "Expected ~{}, got {}",
            expected,
            output.len()
        );
    }

    #[test]
    fn test_multistage_zero_rate() {
        assert!(MultiStageResampler::new(0, 48000).is_err());
        assert!(MultiStageResampler::new(48000, 0).is_err());
    }

    #[test]
    fn test_multistage_empty_input() {
        let mut r = MultiStageResampler::new(4_096_000, 2_048_000).unwrap();
        assert!(r.process(&[]).is_empty());
    }
}
