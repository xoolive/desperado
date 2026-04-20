//! DAB-specific resampling paths tuned for Airspy and RTL front-ends.
//!
//! This module contains the exact rate-matching logic for converting
//! various SDR front-end sample rates to the DAB native rate (2.048 MHz).
//! The paths are carefully chosen to preserve OFDM timing integrity.

use desperado::dsp::resampler::ComplexResampler;
use num_complex::Complex;

use crate::constants;

const WELLE_INTERMEDIATE_RATE: u32 = 4_096_000;

pub(crate) struct DabResampler {
    path: Path,
}

enum Path {
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

impl DabResampler {
    /// Create a DAB resampler for the given SDR input sample rate.
    ///
    /// Uses hardcoded rate ranges tuned for known SDR front-ends:
    /// - 2.048 MHz: passthrough (RTL-SDR native DAB rate)
    /// - ~4.096 MHz: 2:1 averaging (Airspy)
    /// - ~2.4 MHz: direct polyphase resample (RTL-SDR)
    /// - ~3 MHz: resample to 4.096 MHz then halve
    /// - ~6 MHz: halve, resample to 4.096 MHz, halve (Airspy HF+)
    pub(crate) fn new(input_rate_hz: u32) -> Result<Self, String> {
        if input_rate_hz == 0 {
            return Err("Input sample rate must be non-zero".to_string());
        }

        let path = if input_rate_hz == constants::SAMPLE_RATE {
            Path::Passthrough
        } else if (4_000_000..=4_200_000).contains(&input_rate_hz) {
            Path::Half(AveragingDecimator2::new())
        } else if (5_800_000..=6_200_000).contains(&input_rate_hz) {
            let half_rate = input_rate_hz / 2;
            Path::HalfThenResampleThenHalf {
                pre_decimator: AveragingDecimator2::new(),
                resampler: ComplexResampler::new(half_rate, WELLE_INTERMEDIATE_RATE)?,
                post_decimator: AveragingDecimator2::new(),
            }
        } else if (2_900_000..=3_100_000).contains(&input_rate_hz) {
            Path::ResampleThenHalf {
                resampler: ComplexResampler::new(input_rate_hz, WELLE_INTERMEDIATE_RATE)?,
                decimator: AveragingDecimator2::new(),
            }
        } else if (2_300_000..=2_500_000).contains(&input_rate_hz) {
            Path::Direct(ComplexResampler::new(
                input_rate_hz,
                constants::SAMPLE_RATE,
            )?)
        } else {
            return Err(format!(
                "Unsupported input sample rate: {}. Supported: ~2.4M, ~3M, ~4.096M, ~6M",
                input_rate_hz
            ));
        };

        Ok(Self { path })
    }

    pub(crate) fn process(&mut self, input: &[Complex<f32>]) -> Vec<Complex<f32>> {
        if input.is_empty() {
            return Vec::new();
        }

        match &mut self.path {
            Path::Passthrough => input.to_vec(),
            Path::Half(decimator) => decimator.process(input),
            Path::Direct(resampler) => resampler.process(input),
            Path::ResampleThenHalf {
                resampler,
                decimator,
            } => {
                let resampled = resampler.process(input);
                decimator.process(&resampled)
            }
            Path::HalfThenResampleThenHalf {
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

/// Stateful 2:1 averaging decimator for complex samples.
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
